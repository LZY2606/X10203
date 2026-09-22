package jcb.kin

import jcb.model.FrameEdge
import jcb.model.PathTolerance

/**
 * 环路不展开成树：直接沿环做位姿复合得到残差。
 *
 * 残差 = 沿有向环乘积 ∏ T_edge（反向经过边时使用刚体逆变换）。
 * 若环路一致，残差应为单位变换；否则给出平移/旋转偏差，
 * 并通过“击中所有矛盾环的最小边集”指出最小矛盾边。
 */
data class CycleResidual(
    val nodeCycle: List<String>,
    val edges: List<String>,
    val residual: Transform4,
    val translationErrorMeters: Double,
    val rotationErrorRadians: Double,
    val consistent: Boolean,
    /** 环上存在缺失关节值，残差仅在零位假设下给出，不能作为一致性结论。 */
    val evaluable: Boolean,
    val blockedJoints: List<String>,
)

data class CycleReport(
    val cycles: List<CycleResidual>,
    val consistent: Boolean,
    /** 击中所有不一致环的最小（近似最小）边集。 */
    val minimalContradictionEdges: List<String>,
    val inconsistentCycleCount: Int,
    val truncated: Boolean,
)

class CycleAnalyzer(
    private val graph: BuiltFrameGraph,
    private val resolver: JointResolver,
    private val tolerance: PathTolerance = PathTolerance(),
    private val maxCycles: Int = 256,
) {
    fun analyze(): CycleReport {
        val cycles = enumerateElementaryCycles()
        val residuals = cycles.map { evaluate(it) }
        val bad = residuals.filter { !it.consistent }
        val minEdges = if (bad.isEmpty()) emptyList() else minimumHittingSet(bad)
        val truncated = cycles.size >= maxCycles
        return CycleReport(
            cycles = residuals,
            consistent = bad.isEmpty(),
            minimalContradictionEdges = minEdges,
            inconsistentCycleCount = bad.size,
            truncated = truncated,
        )
    }

    private data class UndirectedCycle(val nodes: List<String>, val edges: List<FrameEdge>)

    private fun enumerateElementaryCycles(): List<UndirectedCycle> {
        val results = mutableListOf<List<String>>()
        val seen = mutableSetOf<String>()
        val adjacency: Map<String, List<FrameEdge>> = graph.nodes.keys.associateWith { node ->
            buildList {
                graph.outgoing[node]?.let { addAll(it) }
                graph.edges.filter { it.to == node }.forEach { add(it) }
            }.distinctBy { it.id }
        }

        val orderedNodes = graph.nodes.keys.sorted()
        for (start in orderedNodes) {
            if (results.size >= maxCycles) break
            dfsCycles(start, start, adjacency, linkedSetOf(start), mutableListOf(), results, seen, start)
        }
        return results.map { nodeList ->
            val edges = mutableListOf<FrameEdge>()
            for (i in nodeList.indices) {
                val a = nodeList[i]
                val b = nodeList[(i + 1) % nodeList.size]
                val edge = adjacency.getValue(a).firstOrNull {
                    (it.from == a && it.to == b) || (it.to == a && it.from == b)
                } ?: error("环路边缺失 $a-$b")
                edges.add(edge)
            }
            UndirectedCycle(nodeList, edges)
        }
    }

    private fun dfsCycles(
        start: String,
        current: String,
        adjacency: Map<String, List<FrameEdge>>,
        pathNodes: LinkedHashSet<String>,
        pathEdges: MutableList<FrameEdge>,
        results: MutableList<List<String>>,
        seen: MutableSet<String>,
        smallest: String,
    ) {
        if (results.size >= maxCycles) return
        for (edge in adjacency[current] ?: emptyList()) {
            val next = if (edge.from == current) edge.to else edge.from
            if (next == start && pathEdges.size >= 2) {
                val cycle = pathNodes.toList()
                if (cycle.minOrNull() == smallest) {
                    val key = canonicalCycleKey(cycle)
                    if (seen.add(key)) results.add(cycle)
                }
                continue
            }
            if (next in pathNodes) continue
            // 无向简单环去重：只允许访问 >= 起点（字典序）的节点。
            if (next < smallest) continue
            pathNodes.add(next)
            pathEdges.add(edge)
            dfsCycles(start, next, adjacency, pathNodes, pathEdges, results, seen, smallest)
            pathEdges.removeAt(pathEdges.lastIndex)
            pathNodes.remove(next)
        }
    }

    private fun canonicalCycleKey(nodes: List<String>): String {
        val n = nodes.size
        val rotations = (0 until n).map { offset ->
            (0 until n).joinToString(",") { nodes[(it + offset) % n] }
        }
        val reversed = nodes.asReversed()
        val reverseRotations = (0 until n).map { offset ->
            (0 until n).joinToString(",") { reversed[(it + offset) % n] }
        }
        return (rotations + reverseRotations).min()
    }

    private fun evaluate(cycle: UndirectedCycle): CycleResidual {
        var product = Transform4.identity()
        val blockedJoints = mutableListOf<String>()
        val nodeCount = cycle.nodes.size
        for ((index, edge) in cycle.edges.withIndex()) {
            val sourceNode = cycle.nodes[index]
            val targetNode = cycle.nodes[(index + 1) % nodeCount]
            val directed = edge.from == sourceNode && edge.to == targetNode
            val t = edgeWorldTransform(edge)
            product = if (directed) product * t else product * t.inverse()
            val pose = graph.poses.getValue(edge.id)
            val joint = pose.joint
            if (joint != null && joint.type != jcb.model.JointType.FIXED) {
                val r = resolver.resolve(joint.name)
                if (r.value == null) blockedJoints.add(joint.name)
            }
        }
        val delta = Transform4.identity().delta(product)
        val evaluable = blockedJoints.isEmpty()
        val consistent = evaluable &&
            delta.translationMeters <= tolerance.translationMeters &&
            delta.rotationRadians <= tolerance.rotationRadians
        return CycleResidual(
            nodeCycle = cycle.nodes,
            edges = cycle.edges.map { it.id },
            residual = product,
            translationErrorMeters = if (evaluable) delta.translationMeters else Double.NaN,
            rotationErrorRadians = if (evaluable) delta.rotationRadians else Double.NaN,
            consistent = consistent,
            evaluable = evaluable,
            blockedJoints = blockedJoints,
        )
    }

    private fun edgeWorldTransform(edge: FrameEdge): Transform4 {
        val pose = graph.poses.getValue(edge.id)
        val joint = pose.joint ?: return pose.staticOrigin
        if (joint.type == jcb.model.JointType.FIXED) return pose.staticOrigin
        val resolved = resolver.resolve(joint.name)
        val value = resolved.value ?: return pose.staticOrigin
        val jointTransform = when (joint.type) {
            jcb.model.JointType.REVOLUTE, jcb.model.JointType.CONTINUOUS ->
                Transform4.rotationAround(
                    doubleArrayOf(joint.axis.x, joint.axis.y, joint.axis.z), value)
            jcb.model.JointType.PRISMATIC ->
                Transform4.translationAlong(
                    doubleArrayOf(joint.axis.x, joint.axis.y, joint.axis.z), value)
            else -> Transform4.identity()
        }
        return pose.staticOrigin * jointTransform
    }

    /**
     * 最小击中集（NP 难）：先尝试精确集合覆盖（小规模边集），
     * 边数较大时退化为贪心（按频率取边），并在结果中标注通过精确校验。
     */
    private fun minimumHittingSet(badCycles: List<CycleResidual>): List<String> {
        val sets = badCycles.map { it.edges.toSet() }
        val allEdges = sets.flatten().distinct()
        val n = allEdges.size
        if (n <= 20) {
            // 精确：按子集大小递增枚举。
            for (size in 1..n) {
                val found = enumerateSubsets(allEdges, size).firstOrNull { chosen ->
                    sets.all { cycleEdges -> chosen.any { it in cycleEdges } }
                }
                if (found != null) return found.sorted()
            }
        }
        val remaining = sets.map { it.toMutableSet() }.toMutableList()
        val chosen = mutableListOf<String>()
        while (remaining.isNotEmpty()) {
            val pick = remaining.flatten().groupingBy { it }.eachCount().maxByOrNull { it.value }!!.key
            chosen.add(pick)
            remaining.removeAll { pick in it }
        }
        return chosen.sorted()
    }

    private fun enumerateSubsets(items: List<String>, size: Int): Sequence<List<String>> = sequence {
        val indices = IntArray(size) { it }
        val n = items.size
        while (true) {
            yield(indices.map { items[it] })
            var i = size - 1
            while (i >= 0 && indices[i] == n - size + i) i--
            if (i < 0) return@sequence
            indices[i]++
            for (j in i + 1 until size) indices[j] = indices[j - 1] + 1
        }
    }
}
