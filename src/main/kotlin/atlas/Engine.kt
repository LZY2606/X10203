package atlas

enum class EdgeKind { FIXED, REVOLUTE, CONTINUOUS, PRISMATIC, CALIB }

data class Edge(
    val id: String,
    val from: String,
    val to: String,
    val kind: EdgeKind,
    val source: String,
    val origin: Mat4,
    val axis: DoubleArray,
    val jointName: String?,
    val mimic: Mimic?
)

/** 标定变换：独立版本化，仅在指定序列号与 [validFrom, validTo) 时间段生效。 */
data class CalibTransform(
    val id: String,
    val version: Int,
    val from: String,
    val to: String,
    val xyz: DoubleArray,
    val rpy: DoubleArray,
    val rpyUnit: String,
    val serial: String,
    val validFrom: Long,
    val validTo: Long
) {
    fun rpyRad(): DoubleArray =
        if (rpyUnit == "deg") DoubleArray(3) { Math.toRadians(rpy[it]) } else rpy
}

enum class CalibState { ACTIVE, NOT_YET_VALID, EXPIRED, SERIAL_MISMATCH }

fun calibState(c: CalibTransform, serial: String, time: Long): CalibState = when {
    c.serial != serial -> CalibState.SERIAL_MISMATCH
    time < c.validFrom -> CalibState.NOT_YET_VALID
    time >= c.validTo -> CalibState.EXPIRED
    else -> CalibState.ACTIVE
}

data class JointUse(val joint: String, val value: Double, val unit: String, val status: String)

data class Candidate(
    val frames: List<String>,
    val sources: List<String>,
    val joints: List<JointUse>,
    val matrix: Mat4
)

data class QueryResult(
    val from: String,
    val to: String,
    val candidates: List<Candidate>,
    val pathError: Double?,
    val diagnostics: List<String>
)

data class CycleReport(
    val frames: List<String>,
    val edgeIds: List<String>,
    val residual: Double,
    val consistent: Boolean
)

data class CycleAnalysis(val cycles: List<CycleReport>, val minContradictionEdges: List<String>)

class Engine(
    val urdf: UrdfModel,
    calibs: List<CalibTransform>,
    val serial: String,
    val time: Long
) {
    val diagnostics = mutableListOf<String>()
    val edges = mutableListOf<Edge>()
    private val jointByName = urdf.joints.associateBy { it.name }

    init {
        for (j in urdf.joints) {
            val kind = when (j.type) {
                "fixed" -> EdgeKind.FIXED
                "revolute" -> EdgeKind.REVOLUTE
                "continuous" -> EdgeKind.CONTINUOUS
                "prismatic" -> EdgeKind.PRISMATIC
                else -> EdgeKind.FIXED
            }
            edges.add(
                Edge(
                    "urdf:${j.name}", j.parent, j.child, kind, "urdf:joint:${j.name}",
                    Mat4.fromXyzRpy(j.xyz, j.rpy), j.axis, j.name, j.mimic
                )
            )
        }
        for (c in calibs) {
            when (calibState(c, serial, time)) {
                CalibState.ACTIVE -> edges.add(
                    Edge(
                        "calib:${c.id}@${c.version}", c.from, c.to, EdgeKind.CALIB,
                        "calib:${c.id}:v${c.version}",
                        Mat4.fromXyzRpy(c.xyz, c.rpyRad()), doubleArrayOf(1.0, 0.0, 0.0), null, null
                    )
                )
                else -> diagnostics.add("标定 ${c.id}@v${c.version} 未生效: ${calibState(c, serial, time)}")
            }
        }
        for (d in urdf.duplicateFrames) diagnostics.add("重复 frame: $d")
        val urdfPairs = urdf.joints.map { it.parent to it.child }.toSet()
        for (e in edges.filter { it.kind == EdgeKind.CALIB }) {
            if ((e.from to e.to) in urdfPairs || (e.to to e.from) in urdfPairs) {
                diagnostics.add("变换 ${e.from}->${e.to} 同时由 URDF 与标定声明，保留为并列候选")
            }
        }
    }

    private fun neighbors(frame: String): List<Pair<Edge, Boolean>> =
        edges.mapNotNull { e ->
            when (frame) {
                e.from -> e to true
                e.to -> e to false
                else -> null
            }
        }

    /** 枚举所有简单路径；多条一致路径全部保留为候选，不按遍历次序取舍。 */
    fun allPaths(from: String, to: String, maxDepth: Int = 8, maxPaths: Int = 16): List<List<Pair<Edge, Boolean>>> {
        val out = mutableListOf<List<Pair<Edge, Boolean>>>()
        val visited = mutableSetOf(from)
        fun dfs(cur: String, acc: List<Pair<Edge, Boolean>>) {
            if (out.size >= maxPaths) return
            if (cur == to) {
                if (acc.isNotEmpty()) out.add(acc.toList())
                return
            }
            if (acc.size >= maxDepth) return
            for ((e, fwd) in neighbors(cur)) {
                val nxt = if (fwd) e.to else e.from
                if (nxt in visited) continue
                visited.add(nxt)
                dfs(nxt, acc + (e to fwd))
                visited.remove(nxt)
            }
        }
        dfs(from, emptyList())
        return out
    }

    fun resolveJoint(name: String, values: Map<String, Double>, visiting: Set<String> = emptySet()): Pair<Double, String> {
        values[name]?.let { return it to "OK" }
        val mimic = jointByName[name]?.mimic
        if (mimic != null) {
            if (name in visiting) return 0.0 to "MIMIC_CYCLE"
            val (v, st) = resolveJoint(mimic.joint, values, visiting + name)
            return if (st == "MIMIC_CYCLE") {
                0.0 to "MIMIC_CYCLE"
            } else {
                (mimic.multiplier * v + mimic.offset) to "MIMIC(${mimic.joint})"
            }
        }
        return 0.0 to "MISSING_JOINT_VALUE"
    }

    private fun edgeTransform(e: Edge, values: Map<String, Double>, uses: MutableList<JointUse>): Mat4 =
        when (e.kind) {
            EdgeKind.FIXED, EdgeKind.CALIB -> e.origin
            EdgeKind.REVOLUTE, EdgeKind.CONTINUOUS -> {
                val (q, st) = resolveJoint(e.jointName!!, values)
                uses.add(JointUse(e.jointName, q, "rad", st))
                e.origin * Mat4.axisAngle(e.axis, q)
            }
            EdgeKind.PRISMATIC -> {
                val (q, st) = resolveJoint(e.jointName!!, values)
                uses.add(JointUse(e.jointName, q, "m", st))
                e.origin * Mat4.translation(e.axis[0] * q, e.axis[1] * q, e.axis[2] * q)
            }
        }

    fun query(from: String, to: String, values: Map<String, Double>): QueryResult {
        val paths = allPaths(from, to)
        val diags = mutableListOf<String>()
        if (paths.isEmpty()) diags.add("断链: $from 与 $to 之间无连通路径")
        val candidates = paths.map { path ->
            var m = Mat4.identity()
            val uses = mutableListOf<JointUse>()
            val frames = mutableListOf(from)
            val srcs = mutableListOf<String>()
            for ((e, fwd) in path) {
                val em = edgeTransform(e, values, uses)
                m = m * (if (fwd) em else em.inverseRigid())
                frames.add(if (fwd) e.to else e.from)
                srcs.add(e.source)
            }
            Candidate(frames, srcs, uses, m)
        }
        var err: Double? = null
        if (candidates.size > 1) {
            var d = 0.0
            for (i in candidates.indices) {
                for (j in i + 1 until candidates.size) {
                    d = maxOf(d, candidates[i].matrix.maxAbsDiff(candidates[j].matrix))
                }
            }
            err = d
        }
        return QueryResult(from, to, candidates, err, diags)
    }

    private fun treePath(
        a: String,
        b: String,
        parent: Map<String, Pair<String, Edge>>
    ): List<Pair<Edge, Boolean>>? {
        val upFromA = mutableMapOf<String, List<Pair<Edge, Boolean>>>()
        var cur: String? = a
        var acc = listOf<Pair<Edge, Boolean>>()
        while (cur != null) {
            upFromA[cur] = acc
            val pe = parent[cur] ?: break
            acc = acc + (pe.second to (pe.second.from == cur))
            cur = pe.first
        }
        cur = b
        var accB = listOf<Pair<Edge, Boolean>>()
        while (cur != null) {
            val hit = upFromA[cur]
            if (hit != null) {
                val bPart = accB.reversed().map { (e, fwd) -> e to !fwd }
                return hit + bPart
            }
            val pe = parent[cur] ?: return null
            accB = accB + (pe.second to (pe.second.from == cur))
            cur = pe.first
        }
        return null
    }

    /** 环路不展开成树：对每个基本环计算闭合残差，并给出最小矛盾边集（贪心命中集）。 */
    fun cycles(values: Map<String, Double>, eps: Double = 1e-6): CycleAnalysis {
        val visited = mutableSetOf<String>()
        val parent = mutableMapOf<String, Pair<String, Edge>>()
        val treeEdges = mutableSetOf<String>()
        val nodes = edges.flatMap { listOf(it.from, it.to) }.distinct()
        for (start in nodes) {
            if (start in visited) continue
            visited.add(start)
            val queue = ArrayDeque<String>()
            queue.add(start)
            while (queue.isNotEmpty()) {
                val cur = queue.removeFirst()
                for ((e, fwd) in neighbors(cur)) {
                    val nxt = if (fwd) e.to else e.from
                    if (nxt !in visited) {
                        visited.add(nxt)
                        parent[nxt] = cur to e
                        treeEdges.add(e.id)
                        queue.add(nxt)
                    }
                }
            }
        }
        val reports = mutableListOf<CycleReport>()
        for (e in edges) {
            if (e.id in treeEdges) continue
            val path = treePath(e.from, e.to, parent) ?: continue
            val cyclePath = path + (e to true)
            var m = Mat4.identity()
            val uses = mutableListOf<JointUse>()
            val frames = mutableListOf(e.from)
            for ((ce, fwd) in cyclePath) {
                val em = edgeTransform(ce, values, uses)
                m = m * (if (fwd) em else em.inverseRigid())
                frames.add(if (fwd) ce.to else ce.from)
            }
            val residual = m.maxAbsDiff(Mat4.identity())
            reports.add(CycleReport(frames, cyclePath.map { it.first.id }, residual, residual < eps))
        }
        val uncovered = reports.filter { !it.consistent }.map { it.edgeIds.toSet() }.toMutableList()
        val chosen = mutableListOf<String>()
        while (uncovered.isNotEmpty()) {
            val counts = mutableMapOf<String, Int>()
            for (cyc in uncovered) for (id in cyc) counts.merge(id, 1, Int::plus)
            val best = counts.maxByOrNull { it.value }!!.key
            chosen.add(best)
            uncovered.removeIf { best in it }
        }
        return CycleAnalysis(reports, chosen)
    }
}
