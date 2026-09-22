package atlas

import kotlin.math.abs

/** 现场关节快照：值 + 单位（rad/deg） */
data class JointSnapshot(
    val values: Map<String, Double>,
    val defaultUnit: String = "rad",
    val jointUnits: Map<String, String> = emptyMap(),
) {
    fun unitOf(joint: String) = jointUnits[joint] ?: defaultUnit
    fun rawOf(joint: String) = values[joint]
    fun radOf(joint: String): Double? = values[joint]?.let {
        if (unitOf(joint) == "deg") Math.toRadians(it) else it
    }
}

class Kinematics(
    private val model: RobotModel,
    private val calibrations: List<Calibration>,
) {
    val edges: List<Edge> = buildEdges()
    private val jointByName = model.joints.associateBy { it.name }

    private fun buildEdges(): List<Edge> {
        val out = mutableListOf<Edge>()
        for (j in model.joints) {
            val originM = Mat4.fromXyzRpy(j.origin.xyz, j.origin.rpy)
            val const = if (j.type == JointType.FIXED || j.type == JointType.UNKNOWN) originM else null
            out.add(Edge("urdf:joint:${j.name}", "urdf", j.parent, j.child,
                j.name, j.type, const, j.axis, j.mimic))
        }
        for (c in calibrations) {
            out.add(Edge("cal:${c.id}@v${c.version}", "calibration", c.parentFrame, c.childFrame,
                null, null, Mat4.fromXyzRpy(c.xyz, c.rpy), null, null))
        }
        return out
    }

    /** 解析关节值：处理 mimic 链与 mimic 环 */
    fun resolveJointStates(snapshot: JointSnapshot): Map<String, JointState> {
        val states = mutableMapOf<String, JointState>()
        val visiting = mutableSetOf<String>()
        val stack = mutableListOf<String>()

        fun resolve(name: String): JointState {
            states[name]?.let { return it }
            val j = jointByName[name]
                ?: return JointState.Missing(name).also { states[name] = it }
            if (name in visiting) {
                val cyc = stack.dropWhile { it != name } + name
                val st = JointState.MimicCycle(cyc)
                for (n in cyc) states[n] = st
                return st
            }
            val m = j.mimic
            val st: JointState = if (m == null) {
                val raw = snapshot.rawOf(name)
                if (raw == null) JointState.Missing(name)
                else JointState.Ok(
                    if (snapshot.unitOf(name) == "deg") Math.toRadians(raw) else raw,
                    snapshot.unitOf(name), raw)
            } else {
                visiting.add(name); stack.add(name)
                val ref = resolve(m.joint)
                visiting.remove(name); stack.removeLast()
                when (ref) {
                    is JointState.Ok -> JointState.Ok(ref.valueRad * m.multiplier + m.offset, ref.unit, ref.raw)
                    is JointState.MimicCycle -> ref
                    else -> JointState.MimicMissing(name, m.joint)
                }
            }
            states[name] = st
            return st
        }
        for (j in model.joints) resolve(j.name)
        return states
    }

    /** 单条边在给定关节状态下的变换（正向 parent->child） */
    fun edgeTransform(e: Edge, states: Map<String, JointState>): Pair<Mat4?, JointState?> {
        e.constant?.let { return it to null }
        val jn = e.jointName ?: return null to null
        val st = states[jn] ?: JointState.Missing(jn)
        if (st !is JointState.Ok) return null to st
        val j = jointByName[jn]!!
        val originM = Mat4.fromXyzRpy(j.origin.xyz, j.origin.rpy)
        val motion = when (j.type) {
            JointType.REVOLUTE, JointType.CONTINUOUS -> Mat4.rotationAbout(j.axis, st.valueRad)
            JointType.PRISMATIC -> Mat4.translation(j.axis[0]*st.valueRad, j.axis[1]*st.valueRad, j.axis[2]*st.valueRad)
            else -> Mat4.identity()
        }
        return originM * motion to st
    }

    private fun adjacency(): Map<String, List<Pair<Edge, Boolean>>> {
        val m = mutableMapOf<String, MutableList<Pair<Edge, Boolean>>>()
        for (e in edges) {
            m.getOrPut(e.parent) { mutableListOf() }.add(e to false)
            m.getOrPut(e.child) { mutableListOf() }.add(e to true)
        }
        return m
    }

    /** 枚举 from->to 的全部简单路径（有上限），不偏向遍历次序 */
    fun findAllPaths(from: String, to: String, maxPaths: Int = 16, maxDepth: Int = 24): List<List<Pair<Edge, Boolean>>> {
        if (from == to) return listOf(emptyList())
        val adj = adjacency()
        val out = mutableListOf<List<Pair<Edge, Boolean>>>()
        val visited = mutableSetOf(from)
        val cur = mutableListOf<Pair<Edge, Boolean>>()
        fun dfs(node: String) {
            if (out.size >= maxPaths || cur.size >= maxDepth) return
            for ((e, rev) in adj[node].orEmpty()) {
                val nxt = if (rev) e.parent else e.child
                if (nxt in visited) continue
                cur.add(e to rev)
                if (nxt == to) out.add(cur.toList())
                else { visited.add(nxt); dfs(nxt); visited.remove(nxt) }
                cur.removeLast()
                if (out.size >= maxPaths) return
            }
        }
        dfs(from)
        return out
    }

    fun evalPath(path: List<Pair<Edge, Boolean>>, states: Map<String, JointState>): PathResult {
        var m = Mat4.identity()
        val steps = mutableListOf<PathStep>()
        val problems = mutableListOf<String>()
        var complete = true
        for ((e, rev) in path) {
            val (t, st) = edgeTransform(e, states)
            if (st != null && st !is JointState.Ok) {
                complete = false
                problems.add("${e.id}: ${describeState(st)}")
            }
            if (t == null) { complete = false; continue }
            m = m * (if (rev) t.inverseRigid() else t)
            steps.add(PathStep(e.id, e.source,
                if (rev) e.child else e.parent, if (rev) e.parent else e.child,
                rev, e.jointName, st))
        }
        return PathResult(steps, if (complete || problems.isEmpty()) m else null, complete, problems, 0.0)
    }

    private fun describeState(s: JointState): String = when (s) {
        is JointState.Missing -> "缺少关节值 ${s.joint}"
        is JointState.MimicCycle -> "mimic 环: ${s.joints.joinToString(" -> ")}"
        is JointState.MimicMissing -> "mimic 引用缺失: ${s.joint} 依赖 ${s.ref}"
        is JointState.Ok -> "ok"
    }

    /** 坐标查询：保留全部一致/不一致候选路径 */
    fun queryPose(from: String, to: String, snapshot: JointSnapshot): PoseQueryResult {
        val states = resolveJointStates(snapshot)
        val paths = findAllPaths(from, to)
        if (paths.isEmpty()) {
            return PoseQueryResult(from, to, emptyList(), "disconnected",
                emptyList(), emptyMap())
        }
        val results = paths.map { evalPath(it, states) }.toMutableList()
        // 一致性：与所有完成路径的逐元均值比较
        val complete = results.filter { it.matrix != null }
        if (complete.size > 1) {
            val mean = DoubleArray(16)
            for (r in complete) for (i in 0..15) mean[i] += r.matrix!!.m[i] / complete.size
            val meanM = Mat4(mean)
            for (i in results.indices) {
                val r = results[i]
                if (r.matrix != null) results[i] = r.copy(errorVsConsensus = r.matrix.maxElementDiff(meanM))
            }
        }
        val missing = states.filter { it.value is JointState.Missing }.keys.sorted()
        val units = model.joints.filter { it.mimic == null && snapshot.rawOf(it.name) != null }
            .associate { it.name to snapshot.unitOf(it.name) }
        val status = when {
            results.none { it.complete } -> "incomplete"
            results.size > 1 -> "multiple-candidates"
            else -> "ok"
        }
        return PoseQueryResult(from, to, results, status, missing, units)
    }

    /** 基本环路残差：BFS 生成森林，非树边各定义一个基本环 */
    fun cycleResiduals(states: Map<String, JointState>, tolTrans: Double = 1e-6, tolRotDeg: Double = 1e-4): List<CycleResidual> {
        val adj = adjacency()
        val visited = mutableSetOf<String>()
        val out = mutableListOf<CycleResidual>()
        val frames = edges.flatMap { listOf(it.parent, it.child) }.distinct()
        for (root in frames) {
            if (root in visited) continue
            // BFS 树
            val parent = mutableMapOf<String, Pair<String, Pair<Edge, Boolean>>>() // node -> (prev, edgeStep)
            val queue = ArrayDeque<String>()
            visited.add(root); queue.add(root)
            val treeEdges = mutableSetOf<String>()
            while (queue.isNotEmpty()) {
                val n = queue.removeFirst()
                for ((e, rev) in adj[n].orEmpty()) {
                    val nxt = if (rev) e.parent else e.child
                    if (nxt in visited) continue
                    visited.add(nxt)
                    parent[nxt] = n to (e to rev)
                    treeEdges.add(e.id)
                    queue.add(nxt)
                }
            }
            // 非树边 -> 基本环
            for (e in edges) {
                if (e.id in treeEdges) continue
                // 路径 parent(e)->child(e) 沿树 + 该边
                val pathA = treePath(root, parent, e.parent)
                val pathB = treePath(root, parent, e.child)
                if (pathA == null || pathB == null) continue
                // 残差 = T(root->parent)^-1 ... 直接计算绕环积: root->a, a->b(edge), b->root
                val statesOk = run {
                    var m1 = Mat4.identity(); var ok = true
                    for ((ee, rr) in pathA) { val (t, _) = edgeTransform(ee, states); if (t == null) { ok = false; break }; m1 = m1 * (if (rr) t.inverseRigid() else t) }
                    if (!ok) continue
                    val (te, _) = edgeTransform(e, states); if (te == null) continue
                    var m2 = Mat4.identity()
                    for ((ee, rr) in pathB) { val (t, _) = edgeTransform(ee, states); if (t == null) { ok = false; break }; m2 = m2 * (if (rr) t.inverseRigid() else t) }
                    if (!ok) continue
                    // 绕环: root->a ->(e)-> b ->root(逆)
                    m1 * te * m2.inverseRigid()
                }
                val cycleEdges = (pathA.map { it.first.id } + e.id + pathB.reversed().map { it.first.id }).distinct()
                out.add(CycleResidual(cycleEdges,
                    statesOk.translationErrorVs(Mat4.identity()),
                    statesOk.rotationErrorDegVs(Mat4.identity()),
                    statesOk.translationErrorVs(Mat4.identity()) <= tolTrans &&
                    statesOk.rotationErrorDegVs(Mat4.identity()) <= tolRotDeg))
            }
        }
        return out
    }

    private fun treePath(root: String, parent: Map<String, Pair<String, Pair<Edge, Boolean>>>, node: String): List<Pair<Edge, Boolean>>? {
        if (node == root) return emptyList()
        if (node !in parent) return null
        val out = mutableListOf<Pair<Edge, Boolean>>()
        var cur = node
        while (cur != root) {
            val (prev, step) = parent[cur] ?: return null
            out.add(step)
            cur = prev
        }
        out.reverse()
        return out
    }

    /** 最小矛盾边集：移除后使全部环路一致的最小边集合（先单边后双边，之后贪心） */
    fun minimalContradictionSet(states: Map<String, JointState>): List<String> {
        fun inconsistent(k: Kinematics): Int =
            k.cycleResiduals(states).count { !it.consistent }
        if (inconsistent(this) == 0) return emptyList()
        val edgeIds = edges.map { it.id }
        fun without(ids: Set<String>): Kinematics {
            val urdfJoints = model.joints.filter { "urdf:joint:${it.name}" !in ids }
            val cals = calibrations.filter { "cal:${it.id}@v${it.version}" !in ids }
            return Kinematics(model.copy(joints = urdfJoints), cals)
        }
        for (id in edgeIds) if (inconsistent(without(setOf(id))) == 0) return listOf(id)
        if (edgeIds.size <= 40) {
            for (i in edgeIds.indices) for (j in i+1 until edgeIds.size) {
                val s = setOf(edgeIds[i], edgeIds[j])
                if (inconsistent(without(s)) == 0) return s.toList()
            }
        }
        // 贪心兜底
        val removed = mutableSetOf<String>()
        while (inconsistent(without(removed)) > 0 && removed.size < edgeIds.size) {
            val best = edgeIds.filter { it !in removed }
                .minByOrNull { inconsistent(without(removed + it)) } ?: break
            removed.add(best)
        }
        return removed.toList()
    }
}
