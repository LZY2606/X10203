package jcb.model

import jcb.xml.XmlElement

/** 三维向量/旋转三元组，保留原始字符串以保证导出行的数字风格不变。 */
data class Vec3Spec(val x: Double, val y: Double, val z: Double)

/** URDF <origin xyz rpy>，角度单位固定为弧度（URDF 标准）。 */
data class OriginSpec(
    val xyz: Vec3Spec,
    val rpyRadians: Vec3Spec,
    /** 指向原始 <origin> 元素，用于获批补丁回写。 */
    val element: XmlElement?,
)

data class AxisSpec(val x: Double, val y: Double, val z: Double) {
    companion object {
        val DEFAULT = AxisSpec(1.0, 0.0, 0.0)
    }
}

data class LimitSpec(
    val lower: Double?,
    val upper: Double?,
    val effort: Double?,
    val velocity: Double?,
)

data class MimicSpec(
    val jointName: String,
    val multiplier: Double,
    val offsetRadiansOrMeters: Double,
)

data class InertialSpec(
    val massKg: Double?,
    val ixx: Double, val ixy: Double, val ixz: Double,
    val iyy: Double, val iyz: Double, val izz: Double,
)

sealed class GeometrySpec {
    data class Box(val size: Vec3Spec) : GeometrySpec()
    data class Cylinder(val radius: Double, val length: Double) : GeometrySpec()
    data class Sphere(val radius: Double) : GeometrySpec()
    data class Mesh(val filename: String, val scale: Vec3Spec?) : GeometrySpec()
}

data class VisualCollisionSpec(
    val name: String?,
    val origin: OriginSpec?,
    val geometry: GeometrySpec?,
    val element: XmlElement,
)

enum class JointType { REVOLUTE, CONTINUOUS, PRISMATIC, FIXED, FLOATING, PLANAR, UNKNOWN }

data class LinkSpec(
    val name: String,
    val inertial: InertialSpec?,
    val visuals: List<VisualCollisionSpec>,
    val collisions: List<VisualCollisionSpec>,
    val element: XmlElement,
    /** 同名 link 出现多次时区分序号。 */
    val duplicateIndex: Int,
)

data class JointSpec(
    val name: String,
    val type: JointType,
    val parentLink: String?,
    val childLink: String?,
    val origin: OriginSpec?,
    val axis: AxisSpec,
    val limit: LimitSpec?,
    val mimic: MimicSpec?,
    val element: XmlElement,
    val duplicateIndex: Int,
)

data class UrdfModel(
    val robotName: String?,
    val links: List<LinkSpec>,
    val joints: List<JointSpec>,
    /** 未被识别为 link/joint 的顶层及内嵌元素（保留顺序，往返保真）。 */
    val rootElement: XmlElement,
    val parseWarnings: List<String>,
) {
    fun linkNames(): Set<String> = links.mapTo(mutableSetOf()) { it.name }
}
