package jcb.kin

import jcb.calib.CalibrationEffective
import jcb.model.CalibrationDeclaration
import jcb.model.EdgeOrigin
import jcb.model.FrameEdge
import jcb.model.FrameNode
import jcb.model.JointSpec
import jcb.model.JointType
import jcb.model.NodeSource
import jcb.model.OriginSpec
import jcb.model.UrdfModel
import jcb.model.Vec3Spec

/** 图中一个“运动学边”的位姿定义。 */
data class EdgePose(
    val edgeId: String,
    val staticOrigin: Transform4,
    val joint: JointSpec?,
    val declarationIds: List<String>,
)

data class BuiltFrameGraph(
    val nodes: Map<String, FrameNode>,
    val outgoing: Map<String, List<FrameEdge>>,
    val edges: List<FrameEdge>,
    val poses: Map<String, EdgePose>,
    val warnings: List<String>,
)

object FrameGraphBuilder {

    fun frameKey(name: String, duplicateIndex: Int): String =
        if (duplicateIndex == 0) name else "$name#$duplicateIndex"

    fun build(urdf: UrdfModel, calibration: CalibrationEffective?): BuiltFrameGraph {
        val warnings = urdf.parseWarnings.toMutableList()
        val nodes = linkedMapOf<String, FrameNode>()
        val nameCount = urdf.links.groupingBy { it.name }.eachCount()
        urdf.links.forEach { link ->
            val key = frameKey(link.name, link.duplicateIndex)
            nodes[key] = FrameNode(key, link.name, link.duplicateIndex,
                nameCount[link.name] ?: 1, NodeSource.UrdfLink)
        }

        val edges = mutableListOf<FrameEdge>()
        val poses = mutableMapOf<String, EdgePose>()
        val jointKeyCount = mutableMapOf<String, Int>()

        urdf.joints.forEach { joint ->
            val parent = joint.parentLink
            val child = joint.childLink
            if (parent == null || child == null) {
                warnings.add("joint ${joint.name} 缺少 parent/child，已跳过")
                return@forEach
            }
            val parentKey = linkKey(parent, nodes, urdf)
            val childKey = linkKey(child, nodes, urdf)
            val occ = jointKeyCount.merge("u:$parentKey->$childKey", 0) { a, _ -> a + 1 }!!
            val id = "urdf:${joint.name}" + if (joint.duplicateIndex > 0) "#${joint.duplicateIndex}" else ""
            val origin = joint.origin ?: OriginSpec(Vec3Spec(0.0, 0.0, 0.0), Vec3Spec(0.0, 0.0, 0.0), null)
            edges.add(FrameEdge(id, joint.name, parentKey, childKey,
                EdgeOrigin.URDF, joint, null,
                duplicateDeclarations = occ > 0))
            poses[id] = EdgePose(id, transformOf(origin), joint, emptyList())
        }

        calibration?.declarations?.forEach { decl ->
            ensureCalibrationNode(decl.parentFrame, nodes, calibration.versionId, warnings)
            ensureCalibrationNode(decl.childFrame, nodes, calibration.versionId, warnings)
            val existing = edges.firstOrNull {
                it.from == decl.parentFrame && it.to == decl.childFrame && it.origin == EdgeOrigin.URDF
            }
            val id = "calib:${decl.id}"
            val t = Transform4.origin(decl.xyzMeters, decl.rpyRadians)
            if (existing != null) {
                // 同一变换被 URDF 与标定重复声明：保留两条并列边并显式标记。
                edges.add(FrameEdge(id, "[标定] ${decl.parentFrame}→${decl.childFrame}",
                    decl.parentFrame, decl.childFrame, EdgeOrigin.CALIBRATION, null,
                    calibration.versionId, listOf(decl.id)))
                val oldPose = poses.getValue(existing.id)
                val updated = existing.copy(
                    origin = EdgeOrigin.BOTH,
                    calibrationVersionId = calibration.versionId,
                    calibrationDeclarationIds = existing.calibrationDeclarationIds + decl.id,
                    duplicateDeclarations = true,
                )
                edges[edges.indexOf(existing)] = updated
                poses[updated.id] = oldPose.copy(declarationIds = oldPose.declarationIds + decl.id)
                poses[id] = EdgePose(id, t, null, listOf(decl.id))
                warnings.add("变换 ${decl.parentFrame}→${decl.childFrame} 同时由 URDF(${existing.label}) 与标定(${decl.id}) 声明，保留并列候选")
            } else {
                val dup = edges.any { it.from == decl.parentFrame && it.to == decl.childFrame }
                edges.add(FrameEdge(id, "[标定] ${decl.parentFrame}→${decl.childFrame}",
                    decl.parentFrame, decl.childFrame, EdgeOrigin.CALIBRATION, null,
                    calibration.versionId, listOf(decl.id), duplicateDeclarations = dup))
                poses[id] = EdgePose(id, t, null, listOf(decl.id))
            }
        }

        val outgoing = linkedMapOf<String, MutableList<FrameEdge>>()
        edges.forEach { e ->
            outgoing.getOrPut(e.from) { mutableListOf() }.add(e)
            outgoing.getOrPut(e.to) { mutableListOf() }
        }
        nodes.keys.forEach { outgoing.putIfAbsent(it, mutableListOf()) }
        return BuiltFrameGraph(nodes, outgoing, edges, poses, warnings)
    }

    private fun linkKey(name: String, nodes: Map<String, FrameNode>, urdf: UrdfModel): String {
        nodes[name]?.let { return it.key }
        // 引用不存在的 link：创建同名悬空节点（断链检测会指出）。
        return name
    }

    private fun ensureCalibrationNode(
        name: String,
        nodes: LinkedHashMap<String, FrameNode>,
        versionId: String,
        warnings: MutableList<String>,
    ) {
        if (nodes.containsKey(name)) return
        nodes[name] = FrameNode(name, name, 0, 1, NodeSource.CalibrationOnly(versionId))
        warnings.add("标定引用的帧 $name 在 URDF 中不存在，已作为悬空帧加入")
    }

    private fun transformOf(origin: OriginSpec): Transform4 {
        val xyz = origin.xyz
        val rpy = origin.rpyRadians
        return Transform4.origin(
            doubleArrayOf(xyz.x, xyz.y, xyz.z),
            doubleArrayOf(rpy.x, rpy.y, rpy.z),
        )
    }
}
