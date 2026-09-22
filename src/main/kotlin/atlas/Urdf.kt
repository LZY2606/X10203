package atlas

import org.w3c.dom.Document
import org.w3c.dom.Element
import org.w3c.dom.Node
import java.io.StringReader
import java.io.StringWriter
import javax.xml.parsers.DocumentBuilderFactory
import javax.xml.transform.TransformerFactory
import javax.xml.transform.dom.DOMSource
import javax.xml.transform.stream.StreamResult
import org.xml.sax.InputSource

/**
 * URDF 解析：基于 DOM，保留未知元素、属性与原始顺序。
 * 解析出的运动学模型与原始 Document 并存；导出在未应用获批补丁时
 * 直接序列化原 Document，保证未识别内容往返保留。
 */
object Urdf {

    fun parseDocument(xml: String): Document {
        val f = DocumentBuilderFactory.newInstance()
        f.isNamespaceAware = false
        f.setFeature("http://apache.org/xml/features/disallow-doctype-decl", false)
        val b = f.newDocumentBuilder()
        val d = b.parse(InputSource(StringReader(xml)))
        d.normalize()
        return d
    }

    fun serialize(doc: Document): String {
        val t = TransformerFactory.newInstance().newTransformer()
        t.setOutputProperty(javax.xml.transform.OutputKeys.OMIT_XML_DECLARATION, "no")
        t.setOutputProperty(javax.xml.transform.OutputKeys.INDENT, "no")
        val w = StringWriter()
        t.transform(DOMSource(doc), StreamResult(w))
        return w.toString()
    }

    private fun Element.children(tag: String): List<Element> {
        val out = mutableListOf<Element>()
        val nl = childNodes
        for (i in 0 until nl.length) {
            val n = nl.item(i)
            if (n is Element && n.tagName == tag) out.add(n)
        }
        return out
    }

    private fun Element.child(tag: String): Element? = children(tag).firstOrNull()

    private fun parseDoubles(s: String?, def: DoubleArray): DoubleArray {
        if (s.isNullOrBlank()) return def
        val parts = s.trim().split(Regex("\\s+")).mapNotNull { it.toDoubleOrNull() }
        if (parts.isEmpty()) return def
        val r = def.copyOf()
        for (i in parts.indices) if (i < r.size) r[i] = parts[i]
        return r
    }

    fun parseModel(doc: Document): RobotModel {
        val root = doc.documentElement
        require(root.tagName == "robot") { "根元素必须是 <robot>" }
        val links = root.children("link").map { le ->
            val name = le.getAttribute("name")
            Link(name,
                hasInertial = le.child("inertial") != null,
                hasGeometry = le.child("visual")?.child("geometry") != null ||
                              le.child("collision")?.child("geometry") != null)
        }
        val counts = links.groupingBy { it.name }.eachCount()
        val duplicates = counts.filter { it.value > 1 }.keys.sorted()

        val joints = root.children("joint").map { je ->
            val type = when (je.getAttribute("type") ?: "") {
                "fixed" -> JointType.FIXED
                "revolute" -> JointType.REVOLUTE
                "continuous" -> JointType.CONTINUOUS
                "prismatic" -> JointType.PRISMATIC
                "floating" -> JointType.FLOATING
                "planar" -> JointType.PLANAR
                else -> JointType.UNKNOWN
            }
            val originEl = je.child("origin")
            val origin = Origin(
                xyz = parseDoubles(originEl?.getAttribute("xyz"), doubleArrayOf(0.0, 0.0, 0.0)),
                rpy = parseDoubles(originEl?.getAttribute("rpy"), doubleArrayOf(0.0, 0.0, 0.0)))
            val axis = parseDoubles(je.child("axis")?.getAttribute("xyz"), doubleArrayOf(1.0, 0.0, 0.0))
            val limitEl = je.child("limit")
            val limit = limitEl?.let {
                Limit(it.getAttribute("lower")?.toDoubleOrNull(),
                      it.getAttribute("upper")?.toDoubleOrNull(),
                      it.getAttribute("effort")?.toDoubleOrNull(),
                      it.getAttribute("velocity")?.toDoubleOrNull())
            }
            val mimicEl = je.child("mimic")
            val mimic = mimicEl?.let {
                Mimic(it.getAttribute("joint") ?: "",
                      it.getAttribute("multiplier")?.toDoubleOrNull() ?: 1.0,
                      it.getAttribute("offset")?.toDoubleOrNull() ?: 0.0)
            }
            Joint(
                name = je.getAttribute("name") ?: "",
                type = type,
                parent = je.child("parent")?.getAttribute("link") ?: "",
                child = je.child("child")?.getAttribute("link") ?: "",
                origin = origin, axis = axis, limit = limit, mimic = mimic)
        }
        return RobotModel(root.getAttribute("name") ?: "robot", links, joints, duplicates)
    }

    /** 将获批补丁（joint 元素片段）应用到 Document：同名替换，否则追加到 robot 末尾 */
    fun applyJointPatch(doc: Document, patchXml: String): Int {
        val patchDoc = parseDocument("<patch>$patchXml</patch>")
        val root = doc.documentElement
        var applied = 0
        val nl = patchDoc.documentElement.childNodes
        val toImport = mutableListOf<Element>()
        for (i in 0 until nl.length) {
            val n = nl.item(i)
            if (n is Element && n.tagName == "joint") toImport.add(n)
        }
        for (pj in toImport) {
            val name = pj.getAttribute("name")
            var existing: Element? = null
            val children = root.childNodes
            for (i in 0 until children.length) {
                val n = children.item(i)
                if (n is Element && n.tagName == "joint" && n.getAttribute("name") == name) { existing = n; break }
            }
            val imported = doc.importNode(pj, true) as Element
            if (existing != null) root.replaceChild(imported, existing) else root.appendChild(imported)
            applied++
        }
        return applied
    }

    /** 结构指纹：用于往返保真测试（元素名、属性、顺序的规范化表示） */
    fun fingerprint(doc: Document): String {
        val sb = StringBuilder()
        fun walk(n: Node, depth: Int) {
            when (n) {
                is Element -> {
                    sb.append("  ".repeat(depth)).append('<').append(n.tagName)
                    val attrs = n.attributes
                    val names = (0 until attrs.length).map { attrs.item(it).nodeName }.sorted()
                    for (a in names) sb.append(' ').append(a).append("=\"").append(n.getAttribute(a)).append('"')
                    sb.append(">\n")
                    val c = n.childNodes
                    for (i in 0 until c.length) walk(c.item(i), depth + 1)
                }
                else -> {
                    val t = n.nodeValue?.trim()
                    if (!t.isNullOrEmpty()) sb.append("  ".repeat(depth)).append("text:").append(t).append('\n')
                }
            }
        }
        walk(doc.documentElement, 0)
        return sb.toString()
    }
}
