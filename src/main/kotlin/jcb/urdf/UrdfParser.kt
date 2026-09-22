package jcb.urdf

import jcb.xml.XmlElement
import jcb.xml.parseXml

class UrdfParseException(message: String) : RuntimeException(message)

object UrdfParser {
    fun parse(text: String): Pair<jcb.xml.XmlDocument, UrdfModel> {
        val doc = parseXml(text)
        if (doc.root.tag != "robot") {
            throw UrdfParseException("根元素必须是 <robot>，实际为 <${doc.root.tag}>")
        }
        val root = doc.root
        val links = root.elements("link").map { parseLink(it) }
        val joints = root.elements("joint").map { parseJoint(it) }
        return doc to UrdfModel(root.attr("name"), links, joints)
    }

    private fun parseVector(s: String?): Vector3? =
        s?.trim()?.split(Regex("\\s+"))?.map { it.toDouble() }
            ?.takeIf { it.size == 3 }
            ?.let { Vector3(it[0], it[1], it[2]) }

    private fun parseSix(s: String?): DoubleArray? =
        s?.trim()?.split(Regex("\\s+"))?.map { it.toDouble() }
            ?.takeIf { it.size == 6 }
            ?.let { doubleArrayOf(it[0], it[1], it[2], it[3], it[4], it[5]) }

    private fun parseOrigin(el: XmlElement): PoseSpec {
        val o = el.element("origin") ?: return PoseSpec.IDENTITY
        val xyz = parseVector(o.attr("xyz")) ?: Vector3(0.0, 0.0, 0.0)
        val rpy = parseVector(o.attr("rpy")) ?: Vector3(0.0, 0.0, 0.0)
        val unit = when (o.attr("angle-unit")?.lowercase()) {
            null, "radian", "rad", "radians" -> AngleUnit.RADIAN
            "degree", "deg", "degrees" -> AngleUnit.DEGREE
            else -> AngleUnit.RADIAN
        }
        return PoseSpec(xyz, rpy, unit)
    }

    private fun parseGeometry(g: XmlElement): GeometrySpec? {
        val shape = g.childElements().firstOrNull() ?: return null
        return when (shape.tag) {
            "box" -> GeometrySpec(
                "box", parseVector(shape.attr("size")), null, null, null, null, g
            )
            "sphere" -> GeometrySpec(
                "sphere", null, shape.attr("radius")?.toDoubleOrNull(), null, null, null, g
            )
            "cylinder" -> GeometrySpec(
                "cylinder", null, shape.attr("radius")?.toDoubleOrNull(),
                shape.attr("length")?.toDoubleOrNull(), null, null, g
            )
            "mesh" -> GeometrySpec(
                "mesh", null, null, null, shape.attr("filename"),
                parseVector(shape.attr("scale")), g
            )
            else -> GeometrySpec("unknown", null, null, null, null, null, g)
        }
    }

    private fun parseVisual(el: XmlElement): VisualOrCollision {
        val geom = el.element("geometry")?.let { parseGeometry(it) }
        val pose = el.element("origin")?.let {
            val xyz = parseVector(it.attr("xyz")) ?: Vector3(0.0, 0.0, 0.0)
            val rpy = parseVector(it.attr("rpy")) ?: Vector3(0.0, 0.0, 0.0)
            PoseSpec(xyz, rpy)
        }
        return VisualOrCollision(el.attr("name"), pose, geom)
    }

    private fun parseLink(el: XmlElement): LinkSpec {
        val inertial = el.element("inertial")?.let { ie ->
            val mass = ie.element("mass")?.attr("value")?.toDoubleOrNull()
            val inertia = ie.element("inertia")?.let { i ->
                val keys = listOf("ixx", "ixy", "ixz", "iyy", "iyz", "izz")
                if (keys.all { i.attr(it) != null })
                    doubleArrayOf(*keys.map { i.attr(it)!!.toDouble() }.toDoubleArray())
                else null
            }
            val originEl = ie.element("origin")
            val pose = originEl?.let {
                val xyz = parseVector(it.attr("xyz")) ?: Vector3(0.0, 0.0, 0.0)
                val rpy = parseVector(it.attr("rpy")) ?: Vector3(0.0, 0.0, 0.0)
                PoseSpec(xyz, rpy)
            }
            Inertial(pose, mass, inertia)
        }
        return LinkSpec(
            name = el.attr("name") ?: "(unnamed-link@${el.hashCode()})",
            inertial = inertial,
            visuals = el.elements("visual").map { parseVisual(it) },
            collisions = el.elements("collision").map { parseVisual(it) },
            element = el,
        )
    }

    private fun parseJoint(el: XmlElement): JointSpec {
        val typeRaw = el.attr("type")
        val type = JointType.from(typeRaw)
        val origin = parseOrigin(el)
        val axis = el.element("axis")?.let { parseVector(it.attr("xyz")) }
            ?: Vector3(1.0, 0.0, 0.0)
        val limit = el.element("limit")?.let { l ->
            LimitSpec(
                l.attr("lower")?.toDoubleOrNull(),
                l.attr("upper")?.toDoubleOrNull(),
                l.attr("effort")?.toDoubleOrNull(),
                l.attr("velocity")?.toDoubleOrNull(),
            )
        }
        val mimic = el.element("mimic")?.let { m ->
            MimicSpec(
                jointName = m.attr("joint")
                    ?: throw UrdfParseException("mimic 缺少 joint 属性"),
                multiplier = m.attr("multiplier")?.toDoubleOrNull() ?: 1.0,
                offset = m.attr("offset")?.toDoubleOrNull() ?: 0.0,
            )
        }
        return JointSpec(
            name = el.attr("name") ?: "(unnamed-joint)",
            type = type,
            typeRaw = typeRaw,
            parent = el.element("parent")?.attr("link"),
            child = el.element("child")?.attr("link"),
            origin = origin,
            axis = axis,
            limit = limit,
            mimic = mimic,
            element = el,
        )
    }
}
