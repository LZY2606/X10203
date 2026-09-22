package gsb

import kotlinx.serialization.json.JsonArray
import kotlinx.serialization.json.JsonObject
import kotlinx.serialization.json.jsonArray
import kotlinx.serialization.json.jsonPrimitive
import org.w3c.dom.Document
import org.w3c.dom.Element
import org.w3c.dom.Node
import org.xml.sax.InputSource
import java.io.StringReader
import java.io.StringWriter
import javax.xml.parsers.DocumentBuilderFactory
import javax.xml.transform.TransformerFactory
import javax.xml.transform.dom.DOMSource
import javax.xml.transform.stream.StreamResult

data class Limit(val lower: Double?, val upper: Double?, val effort: Double?, val velocity: Double?)

data class Mimic(val joint: String, val multiplier: Double = 1.0, val offset: Double = 0.0)

enum class JointType {
    FIXED, REVOLUTE, CONTINUOUS, PRISMATIC, FLOATING, PLANAR, UNKNOWN;

    companion object {
        fun of(s: String?): JointType = when (s?.lowercase()) {
            "fixed" -> FIXED
            "revolute" -> REVOLUTE
            "continuous" -> CONTINUOUS
            "prismatic" -> PRISMATIC
            "floating" -> FLOATING
            "planar" -> PLANAR
            else -> UNKNOWN
        }
    }
}

data class Joint(
    val name: String,
    val type: JointType,
    val parentLink: String,
    val childLink: String,
    val origin: Transform,
    val axis: Triple<Double, Double, Double>,
    val limit: Limit?,
    val mimic: Mimic?
) {
    val movable: Boolean
        get() = type == JointType.REVOLUTE || type == JointType.CONTINUOUS || type == JointType.PRISMATIC

    fun motion(q: Double): Transform = when (type) {
        JointType.REVOLUTE, JointType.CONTINUOUS ->
            Transform.rotationAxisAngle(axis.first, axis.second, axis.third, q)
        JointType.PRISMATIC ->
            Transform.translation(axis.first * q, axis.second * q, axis.third * q)
        else -> Transform.identity()
    }

    fun transform(q: Double): Transform = origin * motion(q)
}

data class RobotModel(
    val name: String,
    val links: List<String>,
    val joints: List<Joint>,
    /** Original DOM kept so unknown elements/attributes and document order round-trip on export. */
    val dom: Document
) {
    fun joint(name: String): Joint? = joints.firstOrNull { it.name == name }
}

object Urdf {
    private fun factory(): DocumentBuilderFactory {
        val f = DocumentBuilderFactory.newInstance()
        f.isNamespaceAware = false
        f.isXIncludeAware = false
        f.isExpandEntityReferences = false
        runCatching { f.setFeature("http://apache.org/xml/features/disallow-doctype-decl", true) }
        runCatching { f.setFeature("http://xml.org/sax/features/external-general-entities", false) }
        runCatching { f.setFeature("http://xml.org/sax/features/external-parameter-entities", false) }
        return f
    }

    fun parse(xml: String): RobotModel {
        val doc = factory().newDocumentBuilder().parse(InputSource(StringReader(xml)))
        val root = doc.documentElement ?: throw IllegalArgumentException("empty document")
        require(root.tagName == "robot") { "root element must be <robot>, got <${root.tagName}>" }
        val links = mutableListOf<String>()
        val joints = mutableListOf<Joint>()
        for (child in root.childElements()) {
            when (child.tagName) {
                "link" -> links += child.getAttribute("name")
                "joint" -> joints += parseJoint(child)
                // unknown elements are ignored for the model but stay in the DOM for round-trip
            }
        }
        return RobotModel(root.getAttribute("name") ?: "", links, joints, doc)
    }

    private fun parseJoint(el: Element): Joint {
        var parent = ""
        var child = ""
        var origin = Transform.identity()
        var axis = Triple(1.0, 0.0, 0.0)
        var limit: Limit? = null
        var mimic: Mimic? = null
        for (c in el.childElements()) {
            when (c.tagName) {
                "parent" -> parent = c.getAttribute("link") ?: ""
                "child" -> child = c.getAttribute("link") ?: ""
                "origin" -> {
                    val xyz = parseDoubles(c.getAttribute("xyz"), 3) ?: listOf(0.0, 0.0, 0.0)
                    val rpy = parseDoubles(c.getAttribute("rpy"), 3) ?: listOf(0.0, 0.0, 0.0)
                    origin = Transform.fromXyzRpy(xyz[0], xyz[1], xyz[2], rpy[0], rpy[1], rpy[2])
                }
                "axis" -> {
                    val a = parseDoubles(c.getAttribute("xyz"), 3)
                    if (a != null) axis = Triple(a[0], a[1], a[2])
                }
                "limit" -> limit = Limit(
                    c.attrOrNull("lower"), c.attrOrNull("upper"),
                    c.attrOrNull("effort"), c.attrOrNull("velocity")
                )
                "mimic" -> mimic = Mimic(
                    joint = c.getAttribute("joint") ?: "",
                    multiplier = c.attrOrNull("multiplier") ?: 1.0,
                    offset = c.attrOrNull("offset") ?: 0.0
                )
                // inertial / geometry / unknown children stay untouched in the DOM
            }
        }
        return Joint(
            name = el.getAttribute("name") ?: "",
            type = JointType.of(el.getAttribute("type")),
            parentLink = parent,
            childLink = child,
            origin = origin,
            axis = axis,
            limit = limit,
            mimic = mimic
        )
    }

    fun parseDoubles(s: String?, count: Int): List<Double>? {
        if (s.isNullOrBlank()) return null
        val parts = s.trim().split(Regex("\\s+")).mapNotNull { it.toDoubleOrNull() }
        return if (parts.size == count) parts else null
    }

    private fun Element.attrOrNull(name: String): Double? =
        if (hasAttribute(name)) getAttribute(name).toDoubleOrNull() else null

    fun Element.childElements(): List<Element> {
        val out = mutableListOf<Element>()
        val nl = childNodes
        for (i in 0 until nl.length) {
            val n = nl.item(i)
            if (n.nodeType == Node.ELEMENT_NODE) out += n as Element
        }
        return out
    }

    /** Serialize the (possibly patched) DOM. Unknown content is preserved verbatim. */
    fun serialize(doc: Document): String {
        val t = TransformerFactory.newInstance().newTransformer()
        t.setOutputProperty(javax.xml.transform.OutputKeys.OMIT_XML_DECLARATION, "no")
        t.setOutputProperty(javax.xml.transform.OutputKeys.ENCODING, "UTF-8")
        t.setOutputProperty(javax.xml.transform.OutputKeys.INDENT, "no")
        val w = StringWriter()
        t.transform(DOMSource(doc), StreamResult(w))
        return w.toString()
    }

    private fun jointElement(doc: Document, name: String): Element? {
        val root = doc.documentElement
        for (c in Urdf.run { root.childElements() }) {
            if (c.tagName == "joint" && c.getAttribute("name") == name) return c
        }
        return null
    }

    /**
     * Apply an approved draft patch to a copy of the document.
     * Supported ops: set-origin {joint, xyz?, rpy?}, set-axis {joint, xyz}, set-limit {joint, lower?, upper?}.
     * Unknown ops are rejected so a patch never silently half-applies.
     */
    fun applyPatch(doc: Document, patch: JsonObject): Document {
        val copy = doc.cloneNode(true) as Document
        val ops = (patch["ops"] as? JsonArray) ?: JsonArray(emptyList())
        for (opEl in ops) {
            val op = opEl as JsonObject
            val kind = op["op"]?.jsonPrimitive?.content
                ?: throw IllegalArgumentException("patch op missing 'op'")
            val jointName = op["joint"]?.jsonPrimitive?.content
                ?: throw IllegalArgumentException("patch op missing 'joint'")
            val joint = jointElement(copy, jointName)
                ?: throw IllegalArgumentException("joint '$jointName' not found")
            when (kind) {
                "set-origin" -> {
                    val origin = childByTag(joint, "origin")
                        ?: copy.createElement("origin").also { joint.insertBefore(it, joint.firstChild) }
                    op.xyzList()?.let { origin.setAttribute("xyz", it.joinToString(" ")) }
                    op.rpyList()?.let { origin.setAttribute("rpy", it.joinToString(" ")) }
                }
                "set-axis" -> {
                    val axis = childByTag(joint, "axis")
                        ?: copy.createElement("axis").also { joint.appendChild(it) }
                    val xyz = op.xyzList() ?: throw IllegalArgumentException("set-axis requires xyz")
                    axis.setAttribute("xyz", xyz.joinToString(" "))
                }
                "set-limit" -> {
                    val limit = childByTag(joint, "limit")
                        ?: copy.createElement("limit").also { joint.appendChild(it) }
                    op["lower"]?.jsonPrimitive?.content?.let { limit.setAttribute("lower", it) }
                    op["upper"]?.jsonPrimitive?.content?.let { limit.setAttribute("upper", it) }
                }
                else -> throw IllegalArgumentException("unsupported patch op '$kind'")
            }
        }
        return copy
    }

    private fun childByTag(el: Element, tag: String): Element? =
        Urdf.run { el.childElements() }.firstOrNull { it.tagName == tag }

    private fun JsonObject.xyzList(): List<Double>? =
        (this["xyz"] as? JsonArray)?.jsonArray?.map { it.jsonPrimitive.content.toDouble() }

    private fun JsonObject.rpyList(): List<Double>? =
        (this["rpy"] as? JsonArray)?.jsonArray?.map { it.jsonPrimitive.content.toDouble() }
}
