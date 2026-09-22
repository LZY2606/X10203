package jcb.urdf

import jcb.xml.XmlElement

data class Vector3(val x: Double, val y: Double, val z: Double) {
    fun toArray() = doubleArrayOf(x, y, z)
}

enum class JointType(val urdfName: String) {
    REVOLUTE("revolute"), CONTINUOUS("continuous"), PRISMATIC("prismatic"),
    FIXED("fixed"), FLOATING("floating"), PLANAR("planar"), UNKNOWN("unknown");

    companion object {
        fun from(name: String?): JointType =
            entries.firstOrNull { it.urdfName == name } ?: UNKNOWN
    }
}

data class Inertial(
    val origin: PoseSpec?,
    val massKg: Double?,
    /** Ixx Ixy Ixz Iyy Iyz Izz，单位 kg·m² */
    val inertia: DoubleArray?,
)

data class GeometrySpec(
    val kind: String, // box | sphere | cylinder | mesh | unknown
    val size: Vector3?,
    val radius: Double?,
    val length: Double?,
    val filename: String?,
    val scale: Vector3?,
    val element: XmlElement,
)

data class VisualOrCollision(
    val name: String?,
    val origin: PoseSpec?,
    val geometry: GeometrySpec?,
)

data class LimitSpec(
    val lowerRad: Double?,
    val upperRad: Double?,
    val effort: Double?,
    val velocity: Double?,
)

data class MimicSpec(
    val jointName: String,
    val multiplier: Double,
    val offset: Double,
)

data class PoseSpec(
    val xyz: Vector3 = Vector3(0.0, 0.0, 0.0),
    val rpyRad: Vector3 = Vector3(0.0, 0.0, 0.0),
    /** rpy 解析时使用的角度单位，URDF 标准为 rad。 */
    val angleUnit: AngleUnit = AngleUnit.RADIAN,
) {
    companion object {
        val IDENTITY = PoseSpec()
    }
}

enum class AngleUnit { RADIAN, DEGREE }

data class JointSpec(
    val name: String,
    val type: JointType,
    val typeRaw: String?,
    val parent: String?,
    val child: String?,
    val origin: PoseSpec,
    val axis: Vector3,
    val limit: LimitSpec?,
    val mimic: MimicSpec?,
    val element: XmlElement,
)

data class LinkSpec(
    val name: String,
    val inertial: Inertial?,
    val visuals: List<VisualOrCollision>,
    val collisions: List<VisualOrCollision>,
    val element: XmlElement,
)

data class UrdfModel(
    val robotName: String?,
    val links: List<LinkSpec>,
    val joints: List<JointSpec>,
)
