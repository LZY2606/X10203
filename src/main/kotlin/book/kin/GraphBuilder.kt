package book.kin

import book.calib.CalibrationEdge
import book.calib.CalibrationVersion
import book.math.Mat4
import book.math.Vec3
import book.urdf.Joint
import book.urdf.JointType
import book.urdf.UrdfModel

/**
 * 把 URDF 与一个生效中的标定版本合成为无向多重图。
 * 不做“展开成树”的处理：环路、并行边、断链全部保留并由分析器报告。
 */
object GraphBuilder {

    fun build(
        model: UrdfModel,
        calibration: CalibrationVersion?,
        thresholdMeters: Double = 1e-6,
        thresholdRad: Double = 1e-5,
    ): KinGraph {
        val nodes = mutableListOf<FrameNode>()
        val nameCount = model.links.groupingBy { it.name }.eachCount()

        model.links.forEach { link ->
            val duplicate = (nameCount[link.name] ?: 1) > 1
            val id = if (duplicate) "${link.name}#${link.index}" else link.name
            nodes += FrameNode(id, link.name, link.index, if (duplicate) "urdf(duplicate)" else "urdf")
        }

        // URDF 端点可能悬空；为断链补充外部 frame 节点
        fun nodeFor(frame: String): String {
            val existing = nodes.firstOrNull { it.name == frame }
            if (existing != null) return existing.id
            nodes += FrameNode("external:$frame", frame, -1, "dangling")
            return "external:$frame"
        }

        val edges = mutableListOf<EdgeRef>()
        val overrideJointIds = calibration?.edges
            ?.filter { it.kind == "override" && it.jointName != null }
            ?.map { it.jointName }?.toSet().orEmpty()

        val declaredPairs = HashMap<Pair<String, String>, MutableList<String>>()

        model.joints.forEach { joint ->
            val a = nodeFor(joint.parent)
            val b = nodeFor(joint.child)
            val overridden = joint.name in overrideJointIds
            val calibEdge = calibration?.edges?.firstOrNull {
                it.kind == "override" && it.jointName == joint.name
            }
            val pair = orderedPair(a, b)
            declaredPairs.getOrPut(pair) { mutableListOf() }.add(joint.name)
            edges += EdgeRef(
                id = if (overridden) "calib:${calibEdge?.id}" else "urdf:${joint.name}",
                a = a, b = b,
                kind = if (overridden) "calib-override" else "urdf-joint",
                joint = joint,
                calibEdge = calibEdge,
                sourceLabel = if (overridden)
                    "标定 ${calibEdge?.id} 覆盖 URDF joint ${joint.name}"
                else "URDF joint ${joint.name} (${joint.type.urdfName})",
            )
        }

        calibration?.edges?.filter { it.kind == "new" }?.forEach { ce ->
            val a = nodeFor(ce.parentFrame)
            val b = nodeFor(ce.childFrame)
            val pair = orderedPair(a, b)
            val already = declaredPairs.getOrPut(pair) { mutableListOf() }
            val declaredTwice = already.isNotEmpty()
            already += "calib:${ce.id}"
            edges += EdgeRef(
                id = "calib:${ce.id}", a = a, b = b,
                kind = if (declaredTwice) "duplicate-decl" else "calib-new",
                joint = null, calibEdge = ce,
                sourceLabel = "标定新增边 ${ce.id} (${ce.parentFrame}->${ce.childFrame})" +
                    if (declaredTwice) "；与 ${already.dropLast(1).joinToString()} 重复声明" else "",
                declaredTwice = declaredTwice,
            )
        }

        // 若 override 边与 URDF 构成重复，也标注来源提示（override 是有意覆盖，不算矛盾，
        // 但 new 边与已有 URDF 边同端点则属于“同一变换重复声明”）
        val graph = KinGraph(model, calibration, nodes, edges, thresholdMeters, thresholdRad)
        return graph
    }

    private fun orderedPair(a: String, b: String): Pair<String, String> =
        if (a <= b) a to b else b to a
}

/** 图层面的静态分析：连通分量、断链、重名 frame、环路残差与最小矛盾边集。 */
class GraphAnalyzer(private val graph: KinGraph) {

    fun report(): GraphReport {
        val nodes = graph.frameNodes.keys.toList()
        val visited = HashSet<String>()
        val components = mutableListOf<List<String>>()
        for (n in nodes) {
            if (n in visited) continue
            val comp = mutableListOf<String>()
            val queue = ArrayDeque<String>()
            queue.add(n); visited += n
            while (queue.isNotEmpty()) {
                val cur = queue.removeFirst()
                comp += cur
                graph.edges.filter { it.a == cur || it.b == cur }.forEach { e ->
                    val next = if (e.a == cur) e.b else e.a
                    if (next !in visited) { visited += next; queue.add(next) }
                }
            }
            components += comp
        }

        val broken = graph.frameNodes.values.filter { it.source == "dangling" }
            .map { "外部/缺失 frame '${it.name}'（被关节引用但未在 <link> 中定义）" }
        val duplicateFrames = graph.frameNodes.values.filter { it.source == "urdf(duplicate)" }
            .map { it.name }.distinct()

        val loops = findLoops()
        val inconsistent = loops.filterNot { it.consistent }
        val minContradiction = if (inconsistent.isNotEmpty()) {
            minimumHittingSet(inconsistent.map { lp -> lp.edges.toSet() })
        } else null
        val duplicateDeclarations = graph.edges.filter { it.declaredTwice }
            .map { it.sourceLabel }

        return GraphReport(
            components.map { c -> c.sorted() }.sortedBy { it.firstOrNull() },
            broken, duplicateFrames, loops, minContradiction, duplicateDeclarations,
        )
    }

    /**
     * 枚举基本环路（上限保护），用零关节位形计算闭环残差：
     * T1*T2*...*Tn ≈ I 即一致；否则给定位移/角度残差。
     */
    fun findLoops(maxLoops: Int = 64, maxHops: Int = 14): List<LoopResidual> {
        val adjacency = HashMap<String, MutableList<EdgeRef>>()
        graph.edges.forEach { e ->
            adjacency.getOrPut(e.a) { mutableListOf() }.add(e)
            adjacency.getOrPut(e.b) { mutableListOf() }.add(e)
        }
        val loops = LinkedHashMap<List<String>, LoopResidual>()

        // 以“边顺序”为根：每个无向环的规范表示是其中最小边 id 的边 (u,v)，
        // 从 u 出发沿其它边做简单路径搜索到达 v（不经过根边），与根边闭合。
        val sortedEdges = graph.edges.sortedBy { it.id }
        for ((edgeIdx, rootEdge) in sortedEdges.withIndex()) {
            if (loops.size >= maxLoops) break
            // 根边固定正向 a(parent)->b(child)：DFS 从 b 出发（只走更大 id 的边），
            // 到达 a 后用根边正向闭合。
            val root = rootEdge.b
            val goal = rootEdge.a
            val allowedEdgeIds = sortedEdges.drop(edgeIdx + 1).map { it.id }.toSet()

            fun dfs(cur: String, visited: MutableSet<String>, path: MutableList<EdgeRef>) {
                if (loops.size >= maxLoops || path.size > maxHops) return
                if (cur == goal && path.isNotEmpty()) {
                    // path: root -> ... -> goal；根边保持原始 a/b 方向，
                    // evaluateOriented 在 cur=goal(=child 端 b) 时自动取 T^-1 闭合
                    val cycleEdges = path.toList() + rootEdge
                    val key = canonicalCycle(cycleEdges.map { it.id })
                    if (key !in loops) loops[key] = evaluateOriented(root, goal, cycleEdges)
                    return
                }
                for (e in adjacency[cur].orEmpty()
                    .filter { it.id in allowedEdgeIds }
                    .sortedBy { it.id }) {
                    val next = if (e.a == cur) e.b else e.a
                    if (next in visited) continue
                    visited += next; path.add(e)
                    dfs(next, visited, path)
                    path.removeAt(path.lastIndex); visited -= next
                    if (loops.size >= maxLoops) return
                }
            }
            dfs(root, mutableSetOf(root), mutableListOf())
        }
        return loops.values.sortedWith(
            compareByDescending<LoopResidual> { it.translationError + it.rotationErrorRad }
                .thenBy { it.edges.joinToString() }
        )
    }

    /** cycleEdges 必须形成从 start 出发、沿边链回到 start 的闭环。 */
    private fun evaluateOriented(start: String, goalOfPath: String,
                                 cycleEdges: List<EdgeRef>): LoopResidual {
        var cur = start
        var total = Mat4.identity()
        for (e in cycleEdges) {
            require(cur == e.a || cur == e.b) { "环路边不连续: $cur 不在 ${e.a}/${e.b}" }
            val (t, _) = graph.edgeTransform(e, cur, emptyMap())
            total = total * t
            cur = if (cur == e.a) e.b else e.a
        }
        require(cur == start) { "环路未回到起点（$cur != $start）" }
        val diff = total.diffFrom(Mat4.identity())
        val consistent = diff.translationMeters <= graph.thresholdMeters &&
            diff.rotationRadians <= graph.thresholdRad
        return LoopResidual(
            cycleEdges.map { it.id }, total,
            diff.translationMeters, diff.rotationRadians, consistent,
            graph.thresholdMeters,
        )
    }

    private fun canonicalCycle(edges: List<String>): List<String> {
        val reverses = edges.reversed()
        val a = edges.sorted()
        val b = reverses.sorted()
        return if (a.joinToString() <= b.joinToString()) a else b
    }

    /**
     * 最小矛盾边集 = 覆盖所有不一致环路的最小击中集。
     * 小规模用分支定界求精确解；规模过大退化为贪心并在结果中标注。
     */
    fun minimumHittingSet(cycles: List<Set<String>>, exactLimit: Int = 20): MinimumContradiction {
        val universe = cycles.flatten().toSet()
        if (universe.size <= exactLimit && cycles.size <= 64) {
            val best = branchAndBound(cycles, universe.toList(), 0, emptySet(),
                universe.size + 1)
            if (best != null) {
                return MinimumContradiction(best.sorted(), best.size, "exact-branch-bound",
                    "移除该边集中任意一条不能消除全部矛盾；这是覆盖所有不一致环路的最小组合。")
            }
        }
        // 贪心近似：每次选覆盖最多剩余环的边
        val remaining = cycles.map { it.toMutableSet() }.toMutableList()
        val chosen = sortedSetOf<String>()
        while (remaining.isNotEmpty()) {
            val pick = remaining.flatten().groupingBy { it }.eachCount()
                .maxByOrNull { it.value }?.key ?: break
            chosen += pick
            remaining.removeAll { pick in it }
        }
        return MinimumContradiction(chosen.toList(), chosen.size, "greedy-approx",
            "图规模超过精确解上限，给出贪心近似最小击中集。")
    }

    private fun branchAndBound(
        cycles: List<Set<String>>, edges: List<String>, idx: Int,
        selected: Set<String>, upperBound: Int,
    ): Set<String>? {
        var best: Set<String>? = null
        var bound = upperBound
        val uncovered = cycles.filter { c -> c.none { it in selected } }
        if (uncovered.isEmpty()) return selected
        if (selected.size >= bound || idx >= edges.size) return null

        // 选 edges[idx]
        val withPick = branchAndBound(cycles, edges, idx + 1, selected + edges[idx], bound)
        if (withPick != null && withPick.size < bound) {
            best = withPick; bound = withPick.size
        }
        // 不选 edges[idx]：仅当这条边不选时仍有解的必要才继续
        val withoutPick = branchAndBound(cycles, edges, idx + 1, selected, bound)
        if (withoutPick != null && (best == null || withoutPick.size < best.size)) {
            best = withoutPick
        }
        return best
    }
}
