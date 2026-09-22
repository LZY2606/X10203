package atlas.urdf

import atlas.xml.XmlDocument
import atlas.xml.XmlElement
import atlas.xml.XmlParser

data class Xyz(val x: Double, val y: Double, val z: Double) {
    companion object {
        val ZERO = Xyz(0.0, 0.0, 0.0)
        fun parse(s: String?): Xyz {
            if (s.isNullOrBlank()) return ZERO
            val p = s.trim().split(Regex("\\s+")).map { it.toDouble() }
            require(p.size == 3) { "xyz needs 3 numbers: $s" }
            return Xyz(p[0], p[1], p[2])
        }
    }
}

data class Rpy(val roll: Double, val pitch: Double, val yaw: Double) {
    companion object {
        val ZERO = Rpy(0.0, 0.0, 0.0)
        fun parse(s: String?): Rpy {
            if (s.isNullOrBlank()) return ZERO
            val p = s.trim().split(Regex("\\s+")).map { it.toDouble() }
            require(p.size == 3) { "rpy needs 3 numbers: $s" }
            return Rpy(p[0], p[1], p[2])
        }
    }
}

data class Origin(val xyz: Xyz = Xyz.ZERO, val rpy: Rpy = Rpy.ZERO)

data class Limit(
    val lower: Double? = null,
    val upper: Double? = null,
    val effort: Double? = null,
    val velocity: Double? = null
)

data class Mimic(val joint: String, val multiplier: Double = 1.0, val offset: Double = 0.0)

enum class JointType(val tag: String) {
    REVOLUTE("revolute"), CONTINUOUS("continuous"), PRISMATIC("prismatic"),
    FIXED("fixed"), FLOATING("floating"), PLANAR("planar");

    companion object {
        fun of(tag: String): JointType = entries.firstOrNull { it.tag == tag }
            ?: throw IllegalArgumentException("Unknown joint type: $tag")
    }
}

data class JointModel(
    val name: String,
    val type: JointType,
    val parentLink: String,
    val childLink: String,
    val origin: Origin,
    val axis: Xyz?,
    val limit: Limit?,
    val mimic: Mimic?,
    val element: XmlElement
)

data class LinkModel(
    val name: String,
    val inertial: XmlElement?,
    val geometry: XmlElement?,
    val element: XmlElement
)

class RobotModel(
    val name: String,
    val links: List<LinkModel>,
    val joints: List<JointModel>,
    val document: XmlDocument
) {
    val duplicateLinkNames: List<String> =
        links.groupingBy { it.name }.eachCount().filter { it.value > 1 }.keys.toList()

    fun joint(name: String): JointModel? = joints.firstOrNull { it.name == name }
}

object UrdfParser {
    fun parse(xml: String): RobotModel {
        val doc = XmlParser.parse(xml)
        return fromDocument(doc)
    }

    fun fromDocument(doc: XmlDocument): RobotModel {
        val root = doc.root
        require(root.name == "robot") { "Root element must be <robot>, got <${root.name}>" }
        val links = mutableListOf<LinkModel>()
        val joints = mutableListOf<JointModel>()
        for (child in root.childElements()) {
            when (child.name) {
                "link" -> links.add(parseLink(child))
                "joint" -> joints.add(parseJoint(child))
                // unknown elements stay in the document tree untouched
            }
        }
        return RobotModel(root.attr("name") ?: "robot", links, joints, doc)
    }

    private fun parseLink(el: XmlElement): LinkModel {
        val inertial = el.firstChild("inertial")
        val geometry = el.firstChild("visual")?.firstChild("geometry")
            ?: el.firstChild("collision")?.firstChild("geometry")
        return LinkModel(el.attr("name") ?: "", inertial, geometry, el)
    }

    private fun parseJoint(el: XmlElement): JointModel {
        val type = JointType.of(el.attr("type") ?: "fixed")
        val parent = el.firstChild("parent")?.attr("link")
            ?: throw IllegalArgumentException("joint ${el.attr("name")} missing <parent link=...>")
        val child = el.firstChild("child")?.attr("link")
            ?: throw IllegalArgumentException("joint ${el.attr("name")} missing <child link=...>")
        val originEl = el.firstChild("origin")
        val origin = Origin(
            Xyz.parse(originEl?.attr("xyz")),
            Rpy.parse(originEl?.attr("rpy"))
        )
        val axis = el.firstChild("axis")?.let { Xyz.parse(it.attr("xyz")) }
        val limit = el.firstChild("limit")?.let {
            Limit(
                it.attr("lower")?.toDouble(), it.attr("upper")?.toDouble(),
                it.attr("effort")?.toDouble(), it.attr("velocity")?.toDouble()
            )
        }
        val mimic = el.firstChild("mimic")?.let {
            Mimic(
                it.attr("joint") ?: "",
                it.attr("multiplier")?.toDouble() ?: 1.0,
                it.attr("offset")?.toDouble() ?: 0.0
            )
        }
        return JointModel(
            el.attr("name") ?: "", type, parent, child, origin, axis, limit, mimic, el
        )
    }
}
