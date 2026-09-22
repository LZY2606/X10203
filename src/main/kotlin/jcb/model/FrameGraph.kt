package jcb.model

/** 帧节点：同名 link 重复时用 #序号 区分。 */
data class FrameNode(
    val key: String,
    val name: String,
    val duplicateIndex: Int,
    val duplicateTotal: Int,
    val source: NodeSource,
)

sealed class NodeSource {
    data object UrdfLink : NodeSource()
    /** 标定引用了 URDF 中不存在的帧时产生的悬空节点。 */
    data class CalibrationOnly(val calibrationId: String) : NodeSource()
}

enum class EdgeOrigin { URDF, CALIBRATION, BOTH }

data class FrameEdge(
    val id: String,
    val label: String,
    val from: String,
    val to: String,
    val origin: EdgeOrigin,
    /** URDF joint 引用。 */
    val joint: JointSpec?,
    /** 标定版本 id（若来自标定）。 */
    val calibrationVersionId: String?,
    val calibrationDeclarationIds: List<String> = emptyList(),
    /** 同一 (from,to) 上声明重复时的并列声明（含本边）。 */
    val duplicateDeclarations: Boolean = false,
)
