package jcb

enum class JointType { REVOLUTE, CONTINUOUS, PRISMATIC, FIXED, FLOATING, PLANAR, UNKNOWN }

data class Mimic(val joint: String, val multiplier: Double = 1.0, val offset: Double = 0.0)

data class Limit(val lower: Double?, val upper: Double?, val effort: Double?, val velocity: Double?)

data class JointDef(
    val name: String,
    val type: JointType,
    val parent: String,
    val child: String,
    val xyz: DoubleArray,
    val rpy: DoubleArray,
    val axis: DoubleArray?,
    val mimic: Mimic?,
    val limit: Limit?,
)

data class LinkDef(val name: String, val hasInertial: Boolean, val hasGeometry: Boolean)

data class RobotModel(
    val name: String,
    val links: List<LinkDef>,
    val joints: List<JointDef>,
    val doc: XDocument,
)

object UrdfParser {
    fun parse(xml: String): RobotModel {
        val doc = XmlParser.parse(xml)
        val root = doc.root
        require(root.name == "robot") { "root element must be <robot>, got <${root.name}>" }
        val links = root.elements("link").map { le ->
            LinkDef(
                name = le.attr("name") ?: "",
                hasInertial = le.firstElement("inertial") != null,
                hasGeometry = le.elements("visual").any { it.firstElement("geometry") != null } ||
                    le.elements("collision").any { it.firstElement("geometry") != null },
            )
        }
        val joints = root.elements("joint").map { je ->
            val origin = je.firstElement("origin")
            val limit = je.firstElement("limit")
            val mimic = je.firstElement("mimic")
            JointDef(
                name = je.attr("name") ?: "",
                type = when (je.attr("type") ?: "fixed") {
                    "revolute" -> JointType.REVOLUTE
                    "continuous" -> JointType.CONTINUOUS
                    "prismatic" -> JointType.PRISMATIC
                    "fixed" -> JointType.FIXED
                    "floating" -> JointType.FLOATING
                    "planar" -> JointType.PLANAR
                    else -> JointType.UNKNOWN
                },
                parent = je.firstElement("parent")?.attr("link") ?: "",
                child = je.firstElement("child")?.attr("link") ?: "",
                xyz = parseVec(origin?.attr("xyz"), 0.0),
                rpy = parseVec(origin?.attr("rpy"), 0.0),
                axis = je.firstElement("axis")?.attr("xyz")?.let { parseVec(it, 0.0) },
                mimic = mimic?.let {
                    Mimic(
                        joint = it.attr("joint") ?: "",
                        multiplier = it.attr("multiplier")?.toDoubleOrNull() ?: 1.0,
                        offset = it.attr("offset")?.toDoubleOrNull() ?: 0.0,
                    )
                },
                limit = limit?.let {
                    Limit(
                        lower = it.attr("lower")?.toDoubleOrNull(),
                        upper = it.attr("upper")?.toDoubleOrNull(),
                        effort = it.attr("effort")?.toDoubleOrNull(),
                        velocity = it.attr("velocity")?.toDoubleOrNull(),
                    )
                },
            )
        }
        return RobotModel(root.attr("name") ?: "robot", links, joints, doc)
    }

    private fun parseVec(s: String?, default: Double): DoubleArray {
        if (s == null) return doubleArrayOf(default, default, default)
        val parts = s.trim().split(Regex("\\s+")).map { it.toDoubleOrNull() ?: default }
        return doubleArrayOf(parts.getOrElse(0) { default }, parts.getOrElse(1) { default }, parts.getOrElse(2) { default })
    }
}
