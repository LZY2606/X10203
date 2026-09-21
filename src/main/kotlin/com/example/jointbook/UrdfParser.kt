package com.example.jointbook

import org.w3c.dom.Document
import org.w3c.dom.Element
import org.w3c.dom.Node
import javax.xml.XMLConstants
import javax.xml.parsers.DocumentBuilderFactory
import javax.xml.transform.OutputKeys
import javax.xml.transform.TransformerFactory
import javax.xml.transform.dom.DOMSource
import javax.xml.transform.stream.StreamResult
import java.io.StringReader
import java.io.StringWriter
import org.xml.sax.InputSource

object UrdfParser {
    fun parse(xml: String, id: String, version: Int): UrdfRobot {
        val document = factory().newDocumentBuilder().parse(InputSource(StringReader(xml)))
        document.documentElement.normalize()
        val root = document.documentElement
        require(root.tagName == "robot") { "根元素必须是 robot" }
        val links = mutableListOf<Link>()
        val joints = mutableListOf<Joint>()
        val unknown = mutableListOf<RawDescriptor>()
        childElements(root).forEachIndexed { order, element ->
            when (element.tagName) {
                "link" -> links += parseLink(element, links.count { it.name == element.attr("name") })
                "joint" -> joints += parseJoint(element, joints.count { it.name == element.attr("name") })
                else -> unknown += RawDescriptor(element.tagName, element.attributeMap(), order)
            }
        }
        return UrdfRobot(id, root.attr("name"), version, xml, root, links, joints, unknown)
    }

    fun export(robot: UrdfRobot, approvedPatches: List<XmlPatch> = emptyList()): String {
        val document = factory().newDocumentBuilder().parse(InputSource(StringReader(robot.xml)))
        approvedPatches.filter { it.status == PatchStatus.APPROVED }.forEach { applyPatch(document, it) }
        return serialize(document)
    }

    private fun applyPatch(document: Document, patch: XmlPatch) {
        val nodes = patch.xpath.split('/').filter { it.isNotBlank() }.fold(listOf<Node>(document.documentElement)) { current, segment ->
            current.flatMap { node ->
                if (node !is Element) emptyList() else childElements(node).filter { matchesSegment(it, segment) }
            }
        }
        nodes.forEach { element ->
            if (element is Element) {
                patch.attributes.forEach { (key, value) -> element.setAttribute(key, value) }
                if (patch.text != null) element.textContent = patch.text
            }
        }
    }

    private fun matchesSegment(element: Element, segment: String): Boolean {
        if ('[' !in segment) return element.tagName == segment
        val tag = segment.substringBefore('[')
        val selector = segment.substringAfter('[').removeSuffix("]")
        val (name, value) = selector.split('=', limit = 2).map { it.trim().trim('"') }
        return element.tagName == tag && element.attr(name) == value
    }

    private fun parseLink(element: Element, occurrence: Int): Link {
        var inertial: InertialValue? = null
        val visuals = mutableListOf<VisualLike>()
        val collisions = mutableListOf<VisualLike>()
        val unknown = mutableListOf<RawDescriptor>()
        childElements(element).forEachIndexed { index, child ->
            when (child.tagName) {
                "inertial" -> inertial = parseInertial(child)
                "visual" -> visuals += VisualLike(
                    child.firstElement("origin")?.let(::parseOrigin) ?: Origin(),
                    child.firstElement("geometry")?.let(::parseGeometry),
                    child.attrOrNull("name")
                )
                "collision" -> collisions += VisualLike(
                    child.firstElement("origin")?.let(::parseOrigin) ?: Origin(),
                    child.firstElement("geometry")?.let(::parseGeometry),
                    child.attrOrNull("name")
                )
                else -> unknown += RawDescriptor(child.tagName, child.attributeMap(), index)
            }
        }
        return Link(element.attr("name"), occurrence, inertial, visuals, collisions, unknown)
    }

    private fun parseJoint(element: Element, occurrence: Int): Joint {
        val mimic = childElements(element).firstOrNull { it.tagName == "mimic" }
        val limit = childElements(element).firstOrNull { it.tagName == "limit" }
        val axis = childElements(element).firstOrNull { it.tagName == "axis" }
        return Joint(
            name = element.attr("name"),
            occurrence = occurrence,
            type = when (element.attr("type")) {
                "fixed" -> JointKind.FIXED
                "revolute" -> JointKind.REVOLUTE
                "continuous" -> JointKind.CONTINUOUS
                "prismatic" -> JointKind.PRISMATIC
                "planar" -> JointKind.PLANAR
                "floating" -> JointKind.FLOATING
                else -> JointKind.UNKNOWN
            },
            parent = element.firstElement("parent")?.attr("link") ?: "",
            child = element.firstElement("child")?.attr("link") ?: "",
            origin = element.firstElement("origin")?.let(::parseOrigin) ?: Origin(),
            axis = axis?.let { parseVector(it.attr("xyz"), 3) } ?: listOf(1.0, 0.0, 0.0),
            limit = limit?.let {
                JointLimit(it.doubleAttrOrNull("lower"), it.doubleAttrOrNull("upper"), it.doubleAttrOrNull("effort"), it.doubleAttrOrNull("velocity"))
            },
            mimic = mimic?.let { MimicRule(it.attr("joint"), it.doubleAttr("multiplier", 1.0), it.doubleAttr("offset", 0.0)) },
            unknown = childElements(element).filter { it.tagName !in knownJointChildren }.mapIndexed { index, child ->
                RawDescriptor(child.tagName, child.attributeMap(), index)
            }
        )
    }

    private fun parseInertial(element: Element): InertialValue {
        val mass = element.firstElement("mass")?.doubleAttrOrNull("value")
        val inertia = element.firstElement("inertia")
        return InertialValue(
            element.firstElement("origin")?.let(::parseOrigin) ?: Origin(),
            mass,
            inertia?.doubleAttrOrNull("ixx"),
            inertia?.doubleAttrOrNull("ixy"),
            inertia?.doubleAttrOrNull("ixz"),
            inertia?.doubleAttrOrNull("iyy"),
            inertia?.doubleAttrOrNull("iyz"),
            inertia?.doubleAttrOrNull("izz")
        )
    }

    private fun parseGeometry(element: Element): GeometryDescription? {
        val shape = childElements(element).firstOrNull() ?: return null
        return GeometryDescription(shape.tagName, shape.attributeMap(), shape.textContent.trim().ifBlank { null })
    }

    private fun parseOrigin(element: Element): Origin {
        val angleUnit = when (element.attrOrNull("angle-unit")?.lowercase()) {
            "degree", "deg", "degrees" -> AngleUnit.DEGREE
            else -> AngleUnit.RADIAN
        }
        val linearUnit = when (element.attrOrNull("linear-unit")?.lowercase()) {
            "millimeter", "mm" -> LinearUnit.MILLIMETER
            else -> LinearUnit.METER
        }
        return Origin(
            element.attrOrNull("xyz")?.let { parseVector(it, 3) } ?: listOf(0.0, 0.0, 0.0),
            element.attrOrNull("rpy")?.let { parseVector(it, 3) } ?: listOf(0.0, 0.0, 0.0),
            angleUnit,
            linearUnit
        )
    }

    private fun parseVector(value: String, expected: Int): Vector3 {
        val values = value.trim().split(Regex("\\s+")).map { it.toDouble() }
        require(values.size == expected) { "向量长度必须为 $expected：$value" }
        return values
    }

    private fun serialize(document: Document): String {
        val writer = StringWriter()
        val transformer = TransformerFactory.newInstance().newTransformer()
        transformer.setOutputProperty(OutputKeys.INDENT, "yes")
        transformer.setOutputProperty(OutputKeys.OMIT_XML_DECLARATION, "yes")
        transformer.setOutputProperty("{http://xml.apache.org/xslt}indent-amount", "2")
        transformer.transform(DOMSource(document), StreamResult(writer))
        return writer.toString().trim()
    }

    private fun factory() = DocumentBuilderFactory.newInstance().apply {
        isNamespaceAware = false
        isXIncludeAware = false
        isExpandEntityReferences = false
        setFeature(XMLConstants.FEATURE_SECURE_PROCESSING, true)
        setFeature("http://apache.org/xml/features/disallow-doctype-decl", true)
        setFeature("http://xml.org/sax/features/external-general-entities", false)
        setFeature("http://xml.org/sax/features/external-parameter-entities", false)
        setFeature("http://apache.org/xml/features/nonvalidating/load-external-dtd", false)
    }

    private val knownJointChildren = setOf("parent", "child", "origin", "axis", "limit", "mimic", "dynamics", "calibration", "safety_controller")
}

fun Element.attr(name: String): String = attrOrNull(name) ?: throw IllegalArgumentException("缺少属性 $name")
fun Element.attrOrNull(name: String): String? = getAttribute(name)?.ifBlank { null }
fun Element.doubleAttr(name: String, default: Double): Double = attrOrNull(name)?.toDouble() ?: default
fun Element.doubleAttrOrNull(name: String): Double? = attrOrNull(name)?.toDouble()
fun Element.attributeMap(): Map<String, String> =
    (0 until attributes.length).associate { attributes.item(it).nodeName to attributes.item(it).nodeValue }
fun childElements(element: Element): List<Element> {
    val children = mutableListOf<Element>()
    val nodes = element.childNodes
    for (index in 0 until nodes.length) {
        val node = nodes.item(index)
        if (node.nodeType == Node.ELEMENT_NODE) children += node as Element
    }
    return children
}
fun Element.firstElement(name: String): Element? = childElements(this).firstOrNull { it.tagName == name }
