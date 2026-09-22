package jcb.kin

import jcb.model.FrameEdge
import jcb.model.JointType
import kotlin.math.abs

data class PathStep(
    val edgeId: String,
    val edgeLabel: String,
    val from: String,
    val to: String,
    val jointName: String?,
    val jointType: JointType?,
    val jointValue: Double?,
    val jointUnit: String,
    val jointStatus: JointValueStatus?,
    val jointDetail: String?,
    val staticOrigin: Transform4,
    val jointTransform: Transform4,
    val cumulative: Transform4,
    val source: String,
)

data class PathCandidate(
    val steps: List<PathStep>,
    val transform: Transform4,
    val frames: List<String>,
    val edgeIds: List<String>,
    /** 与其他候选之间的偏差（取所有候选两两比较的最大值）。 */
    var spreadTranslation: Double = 0.0,
    var spreadRotation: Double = 0.0,
    /** 该候选是否处于缺关节值等不可计算状态。 */
    val computable: Boolean,
    val blockedReason: String?,
)

data class QueryResult(
    val fromFrame: String,
    val toFrame: String,
    val connected: Boolean,
    val candidates: List<PathCandidate>,
    /** 选中（误差最小）候选的索引；无可用候选时为 null。不依赖遍历顺序。 */
    val bestCandidateIndex: Int?,
    val tolerance: ModelTolerance,
    val consistent: Boolean?,
    val warnings: List<String>,
)

class PathFinder(
    private val graph: BuiltFrameGraph,
    private val resolver: JointResolver,
    private val tolerance: ModelTolerance = ModelTolerance(),
    /** 简单路径枚举上限，防止稠密图爆炸。 */
    private val maxPaths: Int = 64,
) {
    fun query(from: String, to: String): QueryResult {
        val warnings = mutableListOf<String>()
        if (!graph.nodes.containsKey(from) || !graph.nodes.containsKey(to)) {
            val missing = buildList {
                if (!graph.nodes.containsKey(from)) add(from)
                if (!graph.nodes.containsKey(to)) add(to)
            }
            return QueryResult(from, to, false, emptyList(), null, tolerance, null,
                listOf("帧不存在：${missing.joinToString()}"))
        }
        val rawPaths = enumerateSimplePaths(from, to)
        if (rawPaths.isEmpty()) {
            warnings.add("帧 $from 与 $to 之间断链（不在同一连通分量）")
            return QueryResult(from, to, false, emptyList(), null, tolerance, null, warnings)
        }
        val candidates = rawPaths.map { buildCandidate(it) }
        val computable = candidates.filter { it.computable }
        var bestIndex: Int? = null
        var consistent: Boolean? = null
        if (computable.size >= 2) {
            var maxT = 0.0
            var maxR = 0.0
            for (i in computable.indices) {
                for (j in i + 1 until computable.size) {
                    val delta = computable[i].transform.delta(computable[j].transform)
                    maxT = maxOf(maxT, delta.translationMeters)
                    maxR = maxOf(maxR, delta.rotationRadians)
                }
            }
            computable.forEach { it.spreadTranslation = maxT; it.spreadRotation = maxR }
            consistent = maxT <= tolerance.translationMeters && maxR <= tolerance.rotationRadians
            bestIndex = candidates.indexOf(computable.minByOrNull { candidateScore(it) })
        } else if (computable.size == 1) {
            bestIndex = candidates.indexOf(computable.single())
            consistent = true
        }
        if (candidates.size > 1) {
            warnings.add("存在 ${candidates.size} 条并列路径，全部保留候选；不按遍历顺序选择")
        }
        return QueryResult(from, to, true, candidates, bestIndex, tolerance, consistent, warnings)
    }

    private fun candidateScore(c: PathCandidate): Double {
        // 误差最小优先；并列时用确定性键打破平局，保证结果不依赖遍历次序。
        return c.spreadTranslation * 1e6 + c.spreadRotation
    }

    private fun enumerateSimplePaths(from: String, to: String): List<List<FrameEdge>> {
        val results = mutableListOf<List<FrameEdge>>()
        val visited = mutableSetOf<String>()

        fun dfs(node: String, path: MutableList<FrameEdge>) {
            if (results.size >= maxPaths) return
            if (node == to) {
                results.add(path.toList())
                return
            }
            visited.add(node)
            val edges = (graph.outgoing[node] ?: emptyList())
                .sortedWith(compareBy({ it.to }, { it.id }))
            for (edge in edges) {
                if (edge.to in visited) continue
                path.add(edge)
                dfs(edge.to, path)
                path.removeAt(path.lastIndex)
                if (results.size >= maxPaths) break
            }
            visited.remove(node)
        }

        dfs(from, mutableListOf())
        return results
    }

    private fun buildCandidate(edges: List<FrameEdge>): PathCandidate {
        val steps = mutableListOf<PathStep>()
        var cumulative = Transform4.identity()
        var blocked: String? = null
        val frames = mutableListOf(edges.firstOrNull()?.from ?: "")

        for (edge in edges) {
            val pose = graph.poses.getValue(edge.id)
            val joint = pose.joint
            var jointTransform = Transform4.identity()
            var value: Double? = null
            var status: JointValueStatus? = null
            var detail: String? = null
            var unit = JointResolver.unitText(jcb.kin.JointUnit.NONE)
            if (joint != null && joint.type != JointType.FIXED) {
                val resolved = resolver.resolve(joint.name)
                value = resolved.value
                status = resolved.status
                detail = resolved.detail
                unit = JointResolver.unitText(resolved.unit)
                if (resolved.value == null) {
                    blocked = "关节 ${joint.name} 无可用值：${resolved.detail}"
                } else {
                    jointTransform = when (joint.type) {
                        JointType.REVOLUTE, JointType.CONTINUOUS ->
                            Transform4.rotationAround(
                                doubleArrayOf(joint.axis.x, joint.axis.y, joint.axis.z),
                                resolved.value,
                            )
                        JointType.PRISMATIC ->
                            Transform4.translationAlong(
                                doubleArrayOf(joint.axis.x, joint.axis.y, joint.axis.z),
                                resolved.value,
                            )
                        else -> Transform4.identity()
                    }
                }
            } else if (joint != null) {
                unit = JointResolver.unitText(jcb.kin.JointUnit.NONE)
            }
            cumulative = cumulative * pose.staticOrigin * jointTransform
            steps.add(PathStep(
                edgeId = edge.id,
                edgeLabel = edge.label,
                from = edge.from,
                to = edge.to,
                jointName = joint?.name,
                jointType = joint?.type,
                jointValue = value,
                jointUnit = unit,
                jointStatus = status,
                jointDetail = detail,
                staticOrigin = pose.staticOrigin,
                jointTransform = jointTransform,
                cumulative = cumulative,
                source = sourceText(edge),
            ))
            frames.add(edge.to)
        }
        return PathCandidate(
            steps = steps,
            transform = cumulative,
            frames = frames,
            edgeIds = edges.map { it.id },
            computable = blocked == null,
            blockedReason = blocked,
        )
    }

    private fun sourceText(edge: FrameEdge): String = when (edge.origin) {
        jcb.model.EdgeOrigin.URDF -> "URDF"
        jcb.model.EdgeOrigin.CALIBRATION -> "标定版本 ${edge.calibrationVersionId}"
        jcb.model.EdgeOrigin.BOTH -> "URDF + 标定 ${edge.calibrationVersionId}（重复声明）"
    }
}
