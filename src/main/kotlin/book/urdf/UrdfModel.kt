package book.urdf

import book.math.Vec3
import book.xml.RawDocument
import book.xml.RawElement

enum class JointType(val urdfName: String) {
    REVOLUTE("revolute"), CONTINUOUS("continuous"), PRISMATIC("prismatic"),
    FIXED("fixed"), FLOATING("floating"), PLANAR("planar"), UNKNOWN("unknown");

    companion object {
        fun parse(raw: String?): JointType =
            entries.firstOrNull { it.urdfName == raw } ?: UNKNOWN
    }
}

data class MimicSpec(val jointName: String, val multiplier: Double, val offset: Double)

data class JointLimit(
    val lower: Double?, val upper: Double?,
    val effort: Double?, val velocity: Double?, val rawPresent: Boolean,
)

data class GeometrySpec(
    val kind: String,                       // box / cylinder / sphere / mesh / unknown
    val detail: Map<String, String>,       // 尺寸等原始属性（如 size, radius, filename）
    val element: RawElement,               // 原始 geometry 元素（保真）
)

data class InertialSpec(
    val origin: PoseSpec,
    val mass: Double?,
    /** 3x3 转动惯量，行主序；缺失为 null。 */
    val inertia: Array<DoubleArray>?,
) {
    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (other !is InertialSpec) return false
        return origin == other.origin && mass == other.mass &&
            ((inertia == null && other.inertia == null) ||
                (inertia != null && other.inertia != null && inertia.contentDeepEquals(other.inertia)))
    }
    override fun hashCode(): Int {
        var result = origin.hashCode()
        result = 31 * result + (mass?.hashCode() ?: 0)
        result = 31 * result + (inertia?.contentDeepHashCode() ?: 0)
        return result
    }
}

data class PoseSpec(val xyz: Vec3, val rpy: Vec3) {
    companion object {
        val IDENTITY = PoseSpec(Vec3.ZERO, Vec3.ZERO)
        fun parse(e: RawElement?): PoseSpec {
            if (e == null) return IDENTITY
            return PoseSpec(parseVec(e.attr("xyz"), Vec3.ZERO), parseVec(e.attr("rpy"), Vec3.ZERO))
        }
    }
}

data class Link(
    val name: String,
    val index: Int,
    val inertial: InertialSpec?,
    val visuals: List<GeometrySpec>,
    val collisions: List<GeometrySpec>,
    val element: RawElement,
)

data class Joint(
    val name: String,
    val index: Int,
    val type: JointType,
    val parent: String,
    val child: String,
    val origin: PoseSpec,
    val axis: Vec3,
    val mimic: MimicSpec?,
    val limit: JointLimit,
    val element: RawElement,
) {
    fun isMovable(): Boolean = type == JointType.REVOLUTE || type == JointType.CONTINUOUS ||
        type == JointType.PRISMATIC
}

data class ParseIssue(val level: String, val code: String, val message: String)

data class UrdfModel(
    val robotName: String?,
    val links: List<Link>,
    val joints: List<Joint>,
    val document: RawDocument,
    val issues: List<ParseIssue>,
)

internal fun parseVec(raw: String?, default: Vec3): Vec3 {
    if (raw.isNullOrBlank()) return default
    val parts = raw.trim().split(Regex("\\s+")).mapNotNull { it.toDoubleOrNull() }
    return if (parts.size == 3) Vec3(parts[0], parts[1], parts[2]) else default
}
