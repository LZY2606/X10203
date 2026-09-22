package coordbook

import org.w3c.dom.Document
import org.w3c.dom.Element
import org.w3c.dom.Node
import java.io.StringReader
import java.io.StringWriter
import javax.xml.parsers.DocumentBuilderFactory
import javax.xml.transform.OutputKeys
import javax.xml.transform.TransformerFactory
import javax.xml.transform.dom.DOMSource
import javax.xml.transform.stream.StreamResult
import org.xml.sax.InputSource

data class Origin(val xyz: DoubleArray = doubleArrayOf(0.0, 0.0, 0.0),
                  val rpy: DoubleArray = doubleArrayOf(0.0, 0.0, 0.0)) {
    fun toTransform() = Transform.fromXyzRpy(xyz, rpy)
}

data class Limit(val lower: Double?, val upper: Double?, val effort: Double?, val velocity: Double?)
data class Mimic(val joint: String, val multiplier: Double, val offset: Double)
data class Inertial(val mass: Double?, val origin: Origin?, val inertia: Map<String, Double>)
data class Geometry(val kind: String, val attributes: Map<String, String>)

data class Joint(
    val name: String,
    val type: String,
    val parent: String,
    val child: String,
    val origin: Origin,
    val axis: DoubleArray = doubleArrayOf(1.0, 0.0, 0.0),
    val limit: Limit? = null,
    val mimic: Mimic? = null,
)

data class Link(
    val name: String,
    val inertial: Inertial? = null,
    val geometries: List<Geometry> = emptyList(),
)

data class UrdfRobot(
    val name: String,
    val links: List<Link>,
    val joints: List<Joint>,
    /** Original DOM kept for lossless round-trip of unknown elements/attributes/order. */
    val document: Document,
)

object UrdfParser {
    private val factory = DocumentBuilderFactory.newInstance()

    fun parse(xml: String): UrdfRobot {
        val doc = factory.newDocumentBuilder().parse(InputSource(StringReader(xml)))
        val robot = doc.documentElement
        val links = robot.childElements("link").map { le ->
            val inertial = le.childElements("inertial").firstOrNull()?.let { ie ->
                Inertial(
                    mass = ie.childElements("mass").firstOrNull()?.getAttribute("value")?.toDoubleOrNull(),
                    origin = ie.childElements("origin").firstOrNull()?.let { parseOrigin(it) },
                    inertia = ie.childElements("inertia").firstOrNull()?.let { ine ->
                        ine.attributes().mapNotNull { (k, v) -> v.toDoubleOrNull()?.let { k to it } }.toMap()
                    } ?: emptyMap(),
                )
            }
            val geoms = le.childElements("visual") + le.childElements("collision")
            val parsedGeoms = geoms.mapNotNull { v ->
                v.childElements("geometry").firstOrNull()?.firstElementChild()?.let { g ->
                    Geometry(g.tagName, g.attributes().toMap())
                }
            }
            Link(le.getAttribute("name"), inertial, parsedGeoms)
        }
        val joints = robot.childElements("joint").map { je ->
            val limit = je.childElements("limit").firstOrNull()?.let { l ->
                Limit(
                    l.getAttribute("lower").toDoubleOrNull(), l.getAttribute("upper").toDoubleOrNull(),
                    l.getAttribute("effort").toDoubleOrNull(), l.getAttribute("velocity").toDoubleOrNull(),
                )
            }
            val mimic = je.childElements("mimic").firstOrNull()?.let { mm ->
                Mimic(
                    mm.getAttribute("joint"),
                    mm.getAttribute("multiplier").ifBlank { "1" }.toDouble(),
                    mm.getAttribute("offset").ifBlank { "0" }.toDouble(),
                )
            }
            Joint(
                name = je.getAttribute("name"),
                type = je.getAttribute("type"),
                parent = je.childElements("parent").first().getAttribute("link"),
                child = je.childElements("child").first().getAttribute("link"),
                origin = je.childElements("origin").firstOrNull()?.let { parseOrigin(it) } ?: Origin(),
                axis = je.childElements("axis").firstOrNull()?.let { parseVec(it.getAttribute("xyz")) }
                    ?: doubleArrayOf(1.0, 0.0, 0.0),
                limit = limit,
                mimic = mimic,
            )
        }
        return UrdfRobot(robot.getAttribute("name"), links, joints, doc)
    }

    private fun parseOrigin(e: Element) = Origin(parseVec(e.getAttribute("xyz")), parseVec(e.getAttribute("rpy")))

    private fun parseVec(s: String?): DoubleArray {
        if (s.isNullOrBlank()) return doubleArrayOf(0.0, 0.0, 0.0)
        val parts = s.trim().split(Regex("\\s+")).map { it.toDouble() }
        return DoubleArray(3) { parts.getOrElse(it) { 0.0 } }
    }

    fun serialize(doc: Document): String {
        val t = TransformerFactory.newInstance().newTransformer()
        t.setOutputProperty(OutputKeys.INDENT, "yes")
        t.setOutputProperty(OutputKeys.OMIT_XML_DECLARATION, "no")
        val w = StringWriter()
        t.transform(DOMSource(doc), StreamResult(w))
        return w.toString()
    }
}

fun Element.childElements(tag: String): List<Element> {
    val out = mutableListOf<Element>()
    val kids = childNodes
    for (i in 0 until kids.length) {
        val n = kids.item(i)
        if (n is Element && n.tagName == tag) out.add(n)
    }
    return out
}

fun Element.firstElementChild(): Element? {
    val kids = childNodes
    for (i in 0 until kids.length) if (kids.item(i) is Element) return kids.item(i) as Element
    return null
}

fun Element.attributes(): List<Pair<String, String>> {
    val out = mutableListOf<Pair<String, String>>()
    val attrs = attributes
    for (i in 0 until attrs.length) {
        val n = attrs.item(i)
        out.add(n.nodeName to n.nodeValue)
    }
    return out
}

fun Node.ownerDoc(): Document = if (this is Document) this else ownerDocument
