package jcb.kin

import jcb.urdf.AngleUnit
import jcb.urdf.JointSpec
import jcb.urdf.JointType
import jcb.urdf.UrdfModel
import kotlin.math.atan2
import kotlin.math.sqrt

/** 变换来源。 */
sealed interface EdgeSource {
    val id: String
    val label: String
    /** 是否来自标定（独立版本，不属于原始 URDF）。 */
    val isCalibration: Boolean

    data class UrdfJoint(val jointName: String) : EdgeSource {
        override val id = "urdf:$jointName"
        override val label = "URDF joint $jointName"
        override val isCalibration = false
    }

    data class Calibration(val calibrationId: String, val label0: String) : EdgeSource {
        override val id = "calib:$calibrationId"
        override val label = label0
        override val isCalibration = true
    }
}

data class FrameEdge(
    /** parent 坐标系 */
    val parent: String,
    /** child 坐标系 */
    val child: String,
    val source: EdgeSource,
    /** 零位（joint value = 0）时 T_parent_child */
    val baseTransform: Transform,
    val joint: JointSpec?,
    /** 标定边可为固定；URDF fixed joint 也是固定 */
    val fixed: Boolean,
    /** 声明顺序（用于确定排序，不靠遍历偶然顺序） */
    val declarationOrder: Int,
)

data class GraphIssue(
    val kind: String, // duplicate_frame | broken_link | mimic_chain_unknown | unknown_joint_type
    val severity: String, // error | warning
    val message: String,
    val ref: String? = null,
)

class FrameGraph private constructor(
    val edges: List<FrameEdge>,
    val frames: Set<String>,
    val issues: List<GraphIssue>,
) {
    private val adjacency: Map<String, List<Pair<FrameEdge, Boolean>>> = run {
        val map = LinkedHashMap<String, MutableList<Pair<FrameEdge, Boolean>>>()
        for (e in edges) {
            map.getOrPut(e.parent) { mutableListOf() }.add(e to true)
            map.getOrPut(e.child) { mutableListOf() }.add(e to false)
        }
        map
    }

    fun neighbors(frame: String): List<Pair<FrameEdge, Boolean>> = adjacency[frame] ?: emptyList()

    companion object {
        fun build(
            urdf: UrdfModel,
            calibrationEdges: List<FrameEdge> = emptyList(),
        ): BuildResult {
            val issues = mutableListOf<GraphIssue>()
            val declaredLinks = urdf.links.map { it.name }.toMutableSet()

            // 重复 frame
            urdf.links.groupingBy { it.name }.eachCount().filter { it.value > 1 }.forEach { (name, count) ->
                issues += GraphIssue("duplicate_frame", "error", "link '$name' 重复声明 $count 次", name)
            }
            urdf.joints.groupingBy { it.name }.eachCount().filter { it.value > 1 }.forEach { (name, count) ->
                issues += GraphIssue("duplicate_frame", "error", "joint '$name' 重复声明 $count 次", name)
            }

            val edges = mutableListOf<FrameEdge>()
            var order = 0
            for (j in urdf.joints) {
                val parent = j.parent
                val child = j.child
                if (parent == null || child == null) {
                    issues += GraphIssue(
                        "broken_link", "error",
                        "joint '${j.name}' 缺少 parent/child，形成断链", j.name
                    )
                    continue
                }
                if (parent !in declaredLinks) {
                    issues += GraphIssue(
                        "broken_link", "error",
                        "joint '${j.name}' 的 parent link '$parent' 未声明，断链", parent
                    )
                }
                if (child !in declaredLinks) {
                    issues += GraphIssue(
                        "broken_link", "error",
                        "joint '${j.name}' 的 child link '$child' 未声明，断链", child
                    )
                }
                if (j.type == JointType.UNKNOWN) {
                    issues += GraphIssue(
                        "unknown_joint_type", "warning",
                        "joint '${j.name}' 类型 '${j.typeRaw}' 未识别，按固定关节处理并保留原 XML", j.name
                    )
                }
                val rpy = j.origin.rpyRad
                val (roll, pitch, yaw) = if (j.origin.angleUnit == AngleUnit.DEGREE)
                    Triple(Math.toRadians(rpy.x), Math.toRadians(rpy.y), Math.toRadians(rpy.z))
                else Triple(rpy.x, rpy.y, rpy.z)
                val base = Transform.xyzRpy(
                    j.origin.xyz.toArray(), doubleArrayOf(roll, pitch, yaw)
                )
                edges += FrameEdge(
                    parent, child, EdgeSource.UrdfJoint(j.name), base, j,
                    fixed = j.type == JointType.FIXED || j.type == JointType.UNKNOWN,
                    declarationOrder = order++,
                )
            }

            // mimic 依赖检查（含 mimic 环）
            val jointByName = urdf.joints.associateBy { it.name }
            val mimicDeps = urdf.joints.mapNotNull { j ->
                j.mimic?.let { j.name to it.jointName }
            }.toMap()
            for ((joint, target) in mimicDeps) {
                if (target !in jointByName) {
                    issues += GraphIssue(
                        "mimic_chain_unknown", "error",
                        "mimic: joint '$joint' 跟随的 '$target' 不存在", joint
                    )
                }
            }
            findMimicCycles(mimicDeps).forEach { cycle ->
                issues += GraphIssue(
                    "mimic_cycle", "error",
                    "mimic 形成依赖环：${cycle.joinToString(" -> ")} -> ${cycle.first()}",
                    cycle.firstOrNull()
                )
            }

            val calibIssues = calibrationEdges.mapNotNull { edge ->
                if (edge.parent !in declaredLinks || edge.child !in declaredLinks)
                    GraphIssue(
                        "broken_link", "warning",
                        "标定边 ${edge.source.label} 引用了未声明的 frame ${edge.parent}/${edge.child}",
                        edge.source.id
                    )
                else null
            }
            issues += calibIssues

            // 重复声明：同一 parent/child 对上 URDF 与标定同时存在
            val byPair = (edges + calibrationEdges).groupBy { it.parent to it.child }
                .mapValues { it.value.sortedBy { e -> e.declarationOrder } }
            for ((pair, es) in byPair) {
                if (es.any { it.source.isCalibration } && es.any { !it.source.isCalibration }) {
                    issues += GraphIssue(
                        "duplicate_transform", "warning",
                        "frame ${pair.first} -> ${pair.second} 同时被 URDF(${
                            es.filter { !it.source.isCalibration }.joinToString { it.source.id }
                        }) 与标定(${
                            es.filter { it.source.isCalibration }.joinToString { it.source.id }
                        }) 声明，保留为并列候选",
                        pair.first
                    )
                }
                if (es.count { !it.source.isCalibration } > 1) {
                    issues += GraphIssue(
                        "duplicate_transform", "error",
                        "frame ${pair.first} -> ${pair.second} 在 URDF 中被多个关节声明",
                        pair.first
                    )
                }
            }

            edges += calibrationEdges.sortedBy { it.declarationOrder }
            val frames = edges.flatMap { listOf(it.parent, it.child) }.toSortedSet()
            return BuildResult(FrameGraph(edges, frames, issues), byPair)
        }

        private fun findMimicCycles(deps: Map<String, String>): List<List<String>> {
            val cycles = mutableListOf<List<String>>()
            val seen = mutableSetOf<String>()
            for (start in deps.keys) {
                if (start in seen) continue
                val path = mutableListOf<String>()
                val onPath = mutableSetOf<String>()
                var cur: String? = start
                while (cur != null && deps.containsKey(cur)) {
                    if (cur in onPath) {
                        val cyc = path.drop(path.indexOf(cur))
                        cycles += cyc
                        break
                    }
                    if (cur in seen) break
                    onPath += cur; path += cur
                    cur = deps[cur]
                }
                seen += path
            }
            return cycles
        }
    }

    data class BuildResult(val graph: FrameGraph, val parallelEdges: Map<Pair<String, String>, List<FrameEdge>>)
}
