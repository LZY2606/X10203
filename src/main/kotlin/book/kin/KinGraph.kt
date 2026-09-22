package book.kin

import book.calib.CalibrationEdge
import book.calib.CalibrationVersion
import book.math.Mat4
import book.math.TransformDiff
import book.math.Vec3
import book.urdf.Joint
import book.urdf.JointType
import book.urdf.UrdfModel

/** 一个 frame 出现实例。URDF 中重名 link 会产生多个 occurrence，用 id 区分。 */
data class FrameNode(val id: String, val name: String, val linkIndex: Int, val source: String)

data class EdgeRef(
    val id: String,
    val a: String,
    val b: String,
    val kind: String,                 // urdf-joint / calib-override / calib-new / duplicate-decl
    val joint: Joint?,
    val calibEdge: CalibrationEdge?,
    val sourceLabel: String,
    val declaredTwice: Boolean = false,
)

/** 解析后的关节值（含单位信息）。 */
data class JointValue(
    val jointName: String,
    val value: Double?,
    val unit: String,                 // rad / m
    val providedUnit: String?,        // 调用方传入的单位
    val fromMimic: Boolean,
    val mimicChain: List<String>,
    val status: String,               // OK / MISSING / MIMIC_CYCLE / OUT_OF_LIMIT / UNKNOWN_JOINT
)

data class TracedStep(
    val edgeId: String,
    val fromFrame: String,
    val toFrame: String,
    val reversed: Boolean,
    val transform: Mat4,
    val explanation: String,
    val jointName: String?,
    val jointValue: Double?,
    val unit: String?,
)

data class PathCandidate(
    val index: Int,
    val steps: List<TracedStep>,
    val transform: Mat4,
    val complete: Boolean,
    val problems: List<String>,
    val edgeSources: List<String>,
)

data class PathQueryResult(
    val from: String,
    val to: String,
    val candidates: List<PathCandidate>,
    val chosen: PathCandidate?,
    val spread: TransformDiff?,
    val status: String,
)

data class LoopResidual(
    val edges: List<String>,
    val residualTransform: Mat4,
    val translationError: Double,
    val rotationErrorRad: Double,
    val consistent: Boolean,
    val threshold: Double,
)

data class MinimumContradiction(
    val edgeSet: List<String>,
    val size: Int,
    val method: String,
    val note: String,
)

data class GraphReport(
    val components: List<List<String>>,
    val brokenLinks: List<String>,
    val duplicateFrames: List<String>,
    val loops: List<LoopResidual>,
    val minimumContradiction: MinimumContradiction?,
    val duplicateDeclarations: List<String>,
)

class KinGraph(
    val model: UrdfModel,
    val calibration: CalibrationVersion?,
    nodes: List<FrameNode>,
    val edges: List<EdgeRef>,
    val thresholdMeters: Double,
    val thresholdRad: Double,
) {
    val frameNodes = nodes.associateBy { it.id }

    private val adjacency: Map<String, List<EdgeRef>> = run {
        val map = HashMap<String, MutableList<EdgeRef>>()
        edges.forEach { e ->
            map.getOrPut(e.a) { mutableListOf() }.add(e)
            map.getOrPut(e.b) { mutableListOf() }.add(e)
        }
        map
    }

    fun nodesByName(name: String): List<FrameNode> = frameNodes.values.filter { it.name == name }

    fun resolveFrame(name: String): FrameNode? = nodesByName(name).let {
        when {
            it.isEmpty() -> null
            it.size == 1 -> it.first()
            else -> it.minByOrNull { n -> n.id }
        }
    }

    // ---- 关节值求解（mimic 链） ----

    fun resolveJointValues(inputs: Map<String, Double>, inputUnits: Map<String, String>):
        Map<String, JointValue> {
        val byName = model.joints.associateBy { it.name }
        val result = LinkedHashMap<String, JointValue>()
        val memo = HashMap<String, JointValue>()
        val visiting = HashSet<String>()

        fun unitFor(j: Joint): String =
            if (j.type == JointType.PRISMATIC) "m" else "rad"

        fun resolve(name: String, chain: List<String>): JointValue {
            memo[name]?.let { return it }
            val j = byName[name]
            if (j == null) {
                return JointValue(name, null, "rad", null, false, chain, "UNKNOWN_JOINT")
                    .also { memo[name] = it }
            }
            val mimic = j.mimic
            if (mimic != null) {
                if (mimic.jointName in visiting || mimic.jointName == name) {
                    return JointValue(name, null, unitFor(j), null, true,
                        chain + name, "MIMIC_CYCLE").also { memo[name] = it }
                }
                visiting += name
                val src = if (mimic.jointName in chain) {
                    JointValue(mimic.jointName, null, unitFor(j), null, true,
                        chain + name, "MIMIC_CYCLE")
                } else {
                    resolve(mimic.jointName, chain + name)
                }
                visiting -= name
                val value = if (src.value == null) null else mimic.multiplier * src.value + mimic.offset
                val status = when {
                    src.status == "MIMIC_CYCLE" -> "MIMIC_CYCLE"
                    value == null -> "MISSING"
                    violatesLimit(j, value) -> "OUT_OF_LIMIT"
                    else -> "OK"
                }
                return JointValue(name, value, unitFor(j), src.providedUnit, true,
                    chain + name, status).also { memo[name] = it }
            }
            val raw = inputs[name]
            val provided = inputUnits[name]
            val value = if (raw == null) null else toSi(j, raw, provided)
            val status = when {
                value == null -> "MISSING"
                violatesLimit(j, value) -> "OUT_OF_LIMIT"
                else -> "OK"
            }
            return JointValue(name, value, unitFor(j), provided, false,
                chain + name, status).also { memo[name] = it }
        }

        model.joints.forEach { j ->
            result[j.name] = resolve(j.name, emptyList())
        }
        return result
    }

    private fun violatesLimit(j: Joint, v: Double): Boolean {
        val l = j.limit
        if (!l.rawPresent) return false
        return (l.lower != null && v < l.lower) || (l.upper != null && v > l.upper)
    }

    /** 角度输入允许 deg；内部 SI 为 rad，平移为 m。 */
    private fun toSi(j: Joint, raw: Double, unit: String?): Double {
        if (j.type != JointType.PRISMATIC && (unit == "deg" || unit == "degree")) {
            return Math.toRadians(raw)
        }
        return raw
    }

    // ---- 单边变换 ----

    fun edgeTransform(e: EdgeRef, curFrame: String, values: Map<String, JointValue>):
        Pair<Mat4, String?> {
        // 正向：从 parent(a) 走向 child(b)，用 T；反向：从 child 走向 parent，用 T^-1
        val reversed = curFrame == e.b
        val baseOrigin: Mat4 = when {
            e.calibEdge != null -> {
                val (xyz, rpy) = book.calib.CalibRules.edgeVecs(e.calibEdge)
                Mat4.fromXyzRpy(xyz, rpy)
            }
            e.joint != null -> Mat4.fromXyzRpy(e.joint.origin.xyz, e.joint.origin.rpy)
            else -> Mat4.identity()
        }
        val j = e.joint
        if (j == null || !j.isMovable()) {
            val t = if (reversed) baseOrigin.inverse() else baseOrigin
            return t to null
        }
        val jv = values[j.name]
        if (jv == null || jv.value == null) {
            return (if (reversed) baseOrigin.inverse() else baseOrigin) to
                "关节 ${j.name} 缺值（${jv?.status ?: "MISSING"}），仅按 origin 计算"
        }
        val q = jv.value
        val axis = j.axis
        val motion = if (j.type == JointType.PRISMATIC) {
            Mat4.translation(Vec3(axis.x * q, axis.y * q, axis.z * q))
        } else {
            rotationAbout(axis, q)
        }
        val t = baseOrigin * motion
        return (if (reversed) t.inverse() else t) to null
    }

    // ---- 路径枚举：保留所有简单路径候选，不按遍历顺序选一条 ----

    fun query(from: String, to: String, inputs: Map<String, Double>,
              inputUnits: Map<String, String> = emptyMap(),
              maxPaths: Int = 64, maxHops: Int = 12): PathQueryResult {
        val values = resolveJointValues(inputs, inputUnits)
        val fNode = resolveFrame(from)
        val tNode = resolveFrame(to)
        if (fNode == null || tNode == null) {
            return PathQueryResult(from, to, emptyList(), null, null,
                if (fNode == null && tNode == null) "BOTH_FRAMES_MISSING"
                else if (fNode == null) "FROM_FRAME_MISSING" else "TO_FRAME_MISSING")
        }
        val rawPaths = enumerateSimplePaths(fNode.id, tNode.id, maxPaths, maxHops)
        val candidates = rawPaths.mapIndexed { idx, pathEdges ->
            buildCandidate(idx, fNode.id, tNode.id, pathEdges, values)
        }
        val complete = candidates.filter { it.complete }
        val chosen = selectCandidate(complete)
        val spread = if (complete.size >= 2 && chosen != null) {
            val worstT = complete.maxOf { it.transform.diffFrom(chosen.transform).translationMeters }
            val worstR = complete.maxOf { it.transform.diffFrom(chosen.transform).rotationRadians }
            TransformDiff(worstT, worstR)
        } else null
        val status = when {
            candidates.isEmpty() -> "DISCONNECTED"
            complete.isEmpty() -> "INCOMPLETE"
            complete.size == 1 -> "UNIQUE"
            else -> "MULTIPLE"
        }
        return PathQueryResult(from, to, candidates, chosen, spread, status)
    }

    private fun enumerateSimplePaths(
        start: String, goal: String, maxPaths: Int, maxHops: Int,
    ): List<List<EdgeRef>> {
        val results = mutableListOf<List<EdgeRef>>()
        fun dfs(cur: String, visited: MutableSet<String>, path: MutableList<EdgeRef>) {
            if (results.size >= maxPaths || path.size > maxHops) return
            if (cur == goal && path.isNotEmpty()) {
                results.add(path.toList())
                return
            }
            val neighbors = adjacency[cur].orEmpty().sortedBy { it.id }
            for (e in neighbors) {
                val next = if (e.a == cur) e.b else e.a
                if (next in visited) continue
                visited += next
                path.add(e)
                dfs(next, visited, path)
                path.removeAt(path.lastIndex)
                visited -= next
                if (results.size >= maxPaths) return
            }
        }
        dfs(start, mutableSetOf(start), mutableListOf())
        return results
    }

    private fun buildCandidate(
        idx: Int, startId: String, goalId: String,
        pathEdges: List<EdgeRef>, values: Map<String, JointValue>,
    ): PathCandidate {
        var cur = startId
        var total = Mat4.identity()
        val steps = mutableListOf<TracedStep>()
        val problems = mutableListOf<String>()
        val sources = mutableListOf<String>()
        for (e in pathEdges) {
            val next = if (e.a == cur) e.b else e.a
            val reversed = next == e.a
            val (t, warning) = edgeTransform(e, next, values)
            total = total * t
            val jv = e.joint?.let { values[it.name] }
            if (warning != null) problems += warning
            if (jv != null && e.joint!!.isMovable() && jv.value == null) {
                problems += "关节 ${jv.jointName} 状态 ${jv.status}"
            }
            if (e.declaredTwice) problems += "边 ${e.a}->${e.b} 被 URDF 与标定重复声明"
            sources += e.sourceLabel
            steps += TracedStep(
                e.id, frameNodes.getValue(cur).name, frameNodes.getValue(next).name,
                reversed, t,
                describeEdge(e, reversed, jv),
                e.joint?.name, jv?.value, jv?.unit,
            )
            cur = next
        }
        val complete = problems.isEmpty()
        return PathCandidate(idx, steps, total, complete, problems.distinct(), sources.distinct())
    }

    private fun describeEdge(e: EdgeRef, reversed: Boolean, jv: JointValue?): String {
        val dir = if (reversed) "（逆向）" else ""
        val src = when (e.kind) {
            "calib-override" -> "标定覆盖 ${e.calibEdge?.id}"
            "calib-new" -> "标定新增 ${e.calibEdge?.id}"
            "duplicate-decl" -> "URDF+标定重复声明"
            else -> "URDF joint ${e.joint?.name}"
        }
        val j = if (jv != null && jv.value != null)
            "，q=${"%.6f".format(jv.value)} ${jv.unit}" +
                if (jv.fromMimic) "（mimic 自 ${jv.mimicChain.firstOrNull { it != jv.jointName }}）" else ""
        else ""
        return "$src$dir$j"
    }

    /** 多条一致路径并存：以与中位变换最接近的候选为代表，保留全部候选与误差。 */
    private fun selectCandidate(complete: List<PathCandidate>): PathCandidate? {
        if (complete.isEmpty()) return null
        if (complete.size == 1) return complete.first()
        return complete.minByOrNull { c ->
            complete.sumOf { other ->
                c.transform.diffFrom(other.transform).translationMeters
            }
        }
    }

    companion object {
        fun rotationAbout(axis: Vec3, angle: Double): Mat4 {
            val n = axis.norm().let { if (it == 0.0) 1.0 else it }
            val x = axis.x / n; val y = axis.y / n; val z = axis.z / n
            val c = kotlin.math.cos(angle); val s = kotlin.math.sin(angle)
            val r = Array(4) { DoubleArray(4) }
            r[0][0] = c + x * x * (1 - c)
            r[0][1] = x * y * (1 - c) - z * s
            r[0][2] = x * z * (1 - c) + y * s
            r[1][0] = y * x * (1 - c) + z * s
            r[1][1] = c + y * y * (1 - c)
            r[1][2] = y * z * (1 - c) - x * s
            r[2][0] = z * x * (1 - c) - y * s
            r[2][1] = z * y * (1 - c) + x * s
            r[2][2] = c + z * z * (1 - c)
            r[3][3] = 1.0
            return Mat4(r)
        }
    }
}
