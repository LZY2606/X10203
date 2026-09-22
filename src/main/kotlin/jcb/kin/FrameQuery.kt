package jcb.kin

import jcb.urdf.JointSpec
import jcb.urdf.JointType
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

/** 关节输入：revolute/连续旋转单位可为弧度或度（由 unit 指定），prismatic 单位米。 */
data class JointValue(
    val joint: String,
    val value: Double,
    val unit: JointUnit = JointUnit.RADIAN,
)

enum class JointUnit { RADIAN, DEGREE, METER }

sealed interface QueryStatus {
    data object Ok : QueryStatus
    data class MissingJointValues(val joints: List<String>) : QueryStatus
    data class Unreachable(val reason: String) : QueryStatus
}

data class PathStep(
    val edgeId: String,
    val source: String,
    val from: String,
    val to: String,
    val jointName: String?,
    val jointValueRadOrM: Double?,
    val jointUnit: String?,
    val transform: Transform,
)

data class TransformCandidate(
    val steps: List<PathStep>,
    val transform: Transform,
    val usesCalibration: Boolean,
    val calibrationIds: List<String>,
    val movableJoints: List<String>,
)

data class FrameQueryResult(
    val from: String,
    val to: String,
    val candidates: List<TransformCandidate>,
    val status: QueryStatus,
    val spread: PoseDiff?,
    val connectedComponents: Pair<Boolean, List<String>?> = false to null,
)

class JointResolver(
    private val jointValues: Map<String, JointValue>,
    private val mimic: Map<String, JointSpec>,
) {
    private val mimicCycles = mutableSetOf<String>()
    fun cycleJoints(): Set<String> = mimicCycles
    /** 返回沿一条边可动关节的实际值（弧度或米），null 表示缺失。 */
    fun resolve(edge: FrameEdge): Double? {
        val joint = edge.joint ?: return 0.0
        return resolveJoint(joint, HashSet())
    }

    private fun resolveJoint(joint: JointSpec, visiting: HashSet<String>): Double? {
        if (joint.type == JointType.FIXED) return 0.0
        val direct = jointValues[joint.name]?.let { asCanonical(it, joint.type) }
        if (joint.mimic == null) {
            // 未提供可动关节值 -> 缺失（continuous 无界但仍需显式值，除 0 之外无法假设）
            return direct
        }
        if (direct != null) return direct // 显式值优先
        val leader = mimic[joint.mimic.jointName] ?: return null
        if (!visiting.add(leader.name)) {
            mimicCycles += joint.name
            mimicCycles += leader.name
            return null // mimic 环：没有合法驱动值，报缺失而不是按 0 假装 OK
        }
        val leaderValue = resolveJoint(leader, visiting) ?: return null
        return joint.mimic.multiplier * leaderValue + joint.mimic.offset
    }

    private fun asCanonical(v: JointValue, type: JointType): Double = when (type) {
        JointType.PRISMATIC -> if (v.unit == JointUnit.METER) v.value
        else error("prismatic 关节需要长度单位")
        else -> if (v.unit == JointUnit.DEGREE) Math.toRadians(v.value) else v.value
    }

}

class FrameQueryEngine(
    private val graph: FrameGraph,
    private val maxPaths: Int = 32,
) {
    fun query(
        from: String,
        to: String,
        values: Map<String, JointValue>,
        mimic: Map<String, JointSpec>,
    ): FrameQueryResult {
        if (from == to) {
            val identity = TransformCandidate(
                emptyList(), Transform.identity(), false, emptyList(), emptyList()
            )
            return FrameQueryResult(from, to, listOf(identity), QueryStatus.Ok, PoseDiff(0.0, 0.0))
        }
        if (from !in graph.frames) return FrameQueryResult(
            from, to, emptyList(),
            QueryStatus.Unreachable("frame '$from' 不存在"), null
        )
        if (to !in graph.frames) return FrameQueryResult(
            from, to, emptyList(),
            QueryStatus.Unreachable("frame '$to' 不存在"), null
        )

        val rawPaths = enumeratePaths(from, to)
        if (rawPaths.isEmpty()) {
            return FrameQueryResult(
                from, to, emptyList(),
                QueryStatus.Unreachable("'$from' 与 '$to' 位于不同连通分量（断链）"), null
            )
        }

        val resolver = JointResolver(values, mimic)
        val missing = sortedSetOf<String>()
        val candidates = rawPaths.map { path ->
            var acc = Transform.identity()
            val steps = mutableListOf<PathStep>()
            val calib = mutableListOf<String>()
            val movable = mutableListOf<String>()
            for ((edge, forward) in path) {
                val value = resolver.resolve(edge)
                if (value == null) {
                    edge.joint?.let { missing += it.name }
                }
                edge.joint?.let { missing += resolver.cycleJoints() }
                val t = edgeTransform(edge, value ?: 0.0)
                val oriented = if (forward) t else t.inverse()
                val step = PathStep(
                    edgeId = edge.source.id,
                    source = edge.source.label,
                    from = if (forward) edge.parent else edge.child,
                    to = if (forward) edge.child else edge.parent,
                    jointName = edge.joint?.name,
                    jointValueRadOrM = if (edge.fixed) null else value,
                    jointUnit = when {
                        edge.fixed -> null
                        edge.joint?.type == JointType.PRISMATIC -> "m"
                        else -> "rad"
                    },
                    transform = oriented,
                )
                steps += step
                acc = acc * oriented
                if (edge.source.isCalibration) calib += edge.source.id
                if (!edge.fixed && edge.joint != null) movable += edge.joint.name
            }
            TransformCandidate(steps, acc, calib.isNotEmpty(), calib.distinct(), movable.distinct())
        }.sortedWith(compareBy({ it.movableJoints.size }, { it.steps.size }, {
            it.steps.joinToString(",") { s -> s.edgeId }
        }))

        val status: QueryStatus = if (missing.isEmpty()) QueryStatus.Ok
        else QueryStatus.MissingJointValues(missing.toList())

        val spread = if (candidates.size >= 2) {
            var tMax = 0.0; var rMax = 0.0
            for (i in candidates.indices) for (j in i + 1 until candidates.size) {
                val d = PoseDiff.between(candidates[i].transform, candidates[j].transform)
                tMax = maxOf(tMax, d.translationMeters)
                rMax = maxOf(rMax, d.rotationRadians)
            }
            PoseDiff(tMax, rMax)
        } else PoseDiff(0.0, 0.0)

        return FrameQueryResult(from, to, candidates, status, spread)
    }

    private fun edgeTransform(edge: FrameEdge, valueRadOrM: Double): Transform {
        val j = edge.joint
        if (j == null || edge.fixed) return edge.baseTransform
        val axis = j.axis
        val n = sqrt(axis.x * axis.x + axis.y * axis.y + axis.z * axis.z)
        if (n == 0.0) return edge.baseTransform
        val local = when (j.type) {
            JointType.REVOLUTE, JointType.CONTINUOUS, JointType.UNKNOWN ->
                Transform.quaternion(
                    axis.x / n * sin(valueRadOrM / 2),
                    axis.y / n * sin(valueRadOrM / 2),
                    axis.z / n * sin(valueRadOrM / 2),
                    cos(valueRadOrM / 2),
                )
            JointType.PRISMATIC -> Transform.translation(
                axis.x / n * valueRadOrM, axis.y / n * valueRadOrM, axis.z / n * valueRadOrM
            )
            JointType.FIXED -> Transform.identity()
            JointType.FLOATING, JointType.PLANAR ->
                Transform.identity() // 多自由度需要更多输入，缺失状态由上层体现
        }
        return edge.baseTransform * local
    }

    /**
     * 无向简单路径枚举（按边声明序扩展），返回带方向的边序列。
     * 不依赖遍历偶然顺序：每跳都按 declarationOrder 排序，且在达到上限时整体返回。
     */
    fun enumeratePaths(from: String, to: String): List<List<Pair<FrameEdge, Boolean>>> {
        val results = mutableListOf<List<Pair<FrameEdge, Boolean>>>()
        val visited = HashSet<String>()

        fun dfs(node: String, path: MutableList<Pair<FrameEdge, Boolean>>) {
            if (results.size >= maxPaths) return
            if (node == to) { results += path.toList(); return }
            visited += node
            val nextEdges = graph.neighbors(node).sortedWith(
                compareBy({ it.first.declarationOrder }, { it.first.source.id })
            )
            for ((edge, forward) in nextEdges) {
                val next = if (forward) edge.child else edge.parent
                if (next in visited) continue
                path.add(edge to forward)
                dfs(next, path)
                path.removeAt(path.lastIndex)
                if (results.size >= maxPaths) break
            }
            visited -= node
        }

        dfs(from, mutableListOf())
        return results
    }
}
