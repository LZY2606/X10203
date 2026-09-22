package jcb.kin

/**
 * 环路分析：不把图直接展开成树，而是保留所有边，
 * 用生成树 + 非树边构造基本环（fundamental cycle basis），
 * 计算每个环的闭合残差；对不一致环计算最小矛盾边集（hitting set）。
 */
data class CycleReport(
    val cycles: List<CycleResidual>,
    val inconsistentCycles: List<CycleResidual>,
    val minContradictingEdgeSets: List<List<String>>,
    val note: String,
)

data class CycleResidual(
    val edges: List<String>,          // edge source ids
    val frames: List<String>,         // 环上的 frame（首尾相同）
    val residual: Transform,
    val translationResidualM: Double,
    val rotationResidualRad: Double,
    val consistent: Boolean,
    val toleranceM: Double,
    val toleranceRad: Double,
) {
    fun signature(): String = edges.sorted().joinToString("|")
}

class CycleAnalyzer(
    private val toleranceM: Double = 1e-6,
    private val toleranceRad: Double = 1e-6,
) {
    fun analyze(graph: FrameGraph, values: Map<String, JointValue>, mimic: Map<String, jcb.urdf.JointSpec>): CycleReport {
        // 1. 生成森林：每条无向边只在首次连接两个未连通分量时入树，其余为弦
        val parentOf = HashMap<String, String>()
        val parentEdgeOf = HashMap<String, FrameEdge>()
        val parentFwdOf = HashMap<String, Boolean>()
        val treeEdgeIds = HashSet<String>()
        val seen = HashSet<String>()
        for (rootFrame in graph.frames.sorted()) {
            if (rootFrame in seen) continue
            seen += rootFrame
            val queue = ArrayDeque<String>(); queue.add(rootFrame)
            while (queue.isNotEmpty()) {
                val node = queue.removeFirst()
                for ((edge, forward) in graph.neighbors(node)
                    .sortedWith(compareBy({ it.first.declarationOrder }, { it.first.source.id }))) {
                    val next = if (forward) edge.child else edge.parent
                    if (next in seen) continue
                    seen += next
                    parentOf[next] = node
                    parentEdgeOf[next] = edge
                    parentFwdOf[next] = forward
                    treeEdgeIds += edge.source.id
                    queue.add(next)
                }
            }
        }

        // 2. 弦（非树边），每条边只取一次（其声明方向作为参考方向）
        val chords = graph.edges.filter { it.source.id !in treeEdgeIds }

        fun treePath(a0: String, b0: String): List<Pair<FrameEdge, Boolean>> {
            fun ancestors(x: String): List<String> {
                val list = mutableListOf(x)
                var c: String? = parentOf[x]
                while (c != null) { list += c; c = parentOf[c] }
                return list
            }
            val ancA = ancestors(a0)
            val setB = ancestors(b0).toHashSet()
            val lca = ancA.firstOrNull { it in setB } ?: return emptyList()
            val result = mutableListOf<Pair<FrameEdge, Boolean>>()
            var cur: String = a0
            while (cur != lca) {
                val edge = parentEdgeOf[cur]!!
                // 上行：若树边声明方向为 parent->cur，则与声明反向(false)
                result += edge to (parentFwdOf[cur] != true)
                cur = parentOf[cur]!!
            }
            val down = mutableListOf<Pair<FrameEdge, Boolean>>()
            cur = b0
            while (cur != lca) {
                val edge = parentEdgeOf[cur]!!
                down += edge to (parentFwdOf[cur] == true)
                cur = parentOf[cur]!!
            }
            result += down.reversed()
            return result
        }

        val resolver = JointResolver(values, mimic)
        fun eval(edge: FrameEdge, alongDeclared: Boolean): Transform {
            val v = resolver.resolve(edge) ?: 0.0
            val t = edgeBase(edge, v) // T_parent_child
            return if (alongDeclared) t else t.inverse()
        }

        val cycles = mutableListOf<CycleResidual>()
        for (e in chords) {
            // 从 chord 的 child 沿生成树回到 parent，再沿 chord 声明方向 parent->child 闭合
            val a = e.child
            val b = e.parent
            val tree = treePath(a, b)
            if (tree.isEmpty()) continue
            val ordered = mutableListOf<Pair<FrameEdge, Boolean>>()
            ordered += tree
            ordered += e to true
            var acc = Transform.identity()
            val frames = mutableListOf<String>()
            var node = a
            frames += node
            for ((edge, forward) in ordered) {
                acc = acc * eval(edge, forward)
                node = if (forward) edge.child else edge.parent
                frames += node
            }
            val diff = PoseDiff.between(acc, Transform.identity())
            cycles += CycleResidual(
                edges = ordered.map { it.first.source.id },
                frames = frames,
                residual = acc,
                translationResidualM = diff.translationMeters,
                rotationResidualRad = diff.rotationRadians,
                consistent = diff.translationMeters <= toleranceM && diff.rotationRadians <= toleranceRad,
                toleranceM = toleranceM,
                toleranceRad = toleranceRad,
            )
        }

        val bad = cycles.filter { !it.consistent }
        val (hittingSets, note) = if (bad.isEmpty()) emptyList<List<String>>() to "所有基本环闭合残差均在容差内"
        else minimumHittingSets(bad.map { it.edges.toSet() })

        return CycleReport(
            cycles = cycles.sortedBy { it.signature() },
            inconsistentCycles = bad.sortedBy { it.signature() },
            minContradictingEdgeSets = hittingSets,
            note = note,
        )
    }

    private fun edgeBase(edge: FrameEdge, valueRadOrM: Double): Transform {
        val j = edge.joint
        if (j == null || edge.fixed) return edge.baseTransform
        val axis = j.axis
        val n = kotlin.math.sqrt(axis.x * axis.x + axis.y * axis.y + axis.z * axis.z)
        if (n == 0.0) return edge.baseTransform
        val local = when (j.type) {
            jcb.urdf.JointType.REVOLUTE, jcb.urdf.JointType.CONTINUOUS, jcb.urdf.JointType.UNKNOWN ->
                Transform.quaternion(
                    axis.x / n * kotlin.math.sin(valueRadOrM / 2),
                    axis.y / n * kotlin.math.sin(valueRadOrM / 2),
                    axis.z / n * kotlin.math.sin(valueRadOrM / 2),
                    kotlin.math.cos(valueRadOrM / 2),
                )
            jcb.urdf.JointType.PRISMATIC -> Transform.translation(
                axis.x / n * valueRadOrM, axis.y / n * valueRadOrM, axis.z / n * valueRadOrM
            )
            else -> Transform.identity()
        }
        return edge.baseTransform * local
    }

    /**
     * 最小击中集（击中所有不一致环）。
     * 边数 ≤ 20 时穷举所有最小基数解；否则贪心近似并标注。
     */
    private fun minimumHittingSets(sets: List<Set<String>>): Pair<List<List<String>>, String> {
        val universe = sets.flatten().toSet()
        if (universe.size <= 20) {
            val edges = universe.toList()
            for (k in 1..edges.size) {
                val combos = combinations(edges, k)
                val hits = combos.filter { pick -> sets.all { cycle -> pick.any { it in cycle } } }
                if (hits.isNotEmpty()) {
                    return hits.map { it.sorted() }.sortedBy { it.joinToString() } to
                        "最小矛盾边集（基数 $k，穷举）：移除/修正其中任意一组即可消除所有环路矛盾"
                }
            }
            return emptyList<List<String>>() to "未找到矛盾边集"
        }
        val remaining = sets.map { it.toMutableSet() }.toMutableList()
        val chosen = sortedSetOf<String>()
        while (remaining.any { it.isNotEmpty() }) {
            val pick = remaining.flatten().groupingBy { it }.eachCount()
                .maxWithOrNull(compareBy<Map.Entry<String, Int>> { it.value }.thenByDescending { it.key })!!.key
            chosen += pick
            remaining.forEach { it.remove(pick) }
            remaining.removeAll { it.isEmpty() }
        }
        return listOf(chosen.toList()) to "最小矛盾边集（贪心近似，边数>20 未穷举）"
    }

    private fun <T> combinations(items: List<T>, k: Int): List<List<T>> {
        if (k == 0) return listOf(emptyList())
        if (items.size < k) return emptyList()
        if (k == 1) return items.map { listOf(it) }
        val result = mutableListOf<List<T>>()
        for (i in 0..items.size - k) {
            val head = items[i]
            combinations(items.subList(i + 1, items.size), k - 1).forEach { result += listOf(head) + it }
        }
        return result
    }
}
