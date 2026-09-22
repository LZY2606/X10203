package jcb.xml

object XmlWriter {
    fun renderDocument(doc: XmlDocument): String = buildString {
        doc.prolog.forEach { append(it.raw) }
        appendElement(doc.root, this)
        doc.epilog.forEach { append(it.raw) }
    }

    fun renderElement(el: XmlElement): String = buildString { appendElement(el, this) }

    fun appendElement(el: XmlElement, sb: StringBuilder) {
        sb.append('<').append(el.tag)
        for ((k, v) in el.attributes) {
            sb.append(' ').append(k).append('=').append('"').append(escapeAttr(v)).append('"')
        }
        if (el.children.isEmpty()) { sb.append("/>"); return }
        sb.append('>')
        for (child in el.children) {
            when (child) {
                is XmlElement -> appendElement(child, sb)
                is XmlText -> sb.append(child.raw)
                is XmlRaw -> sb.append(child.raw)
            }
        }
        sb.append("</").append(el.tag).append('>')
    }

    private fun escapeAttr(s: String): String = buildString(s.length) {
        for (c in s) when (c) {
            '<' -> append("&lt;"); '>' -> append("&gt;"); '&' -> append("&amp;")
            '"' -> append("&quot;"); '\n' -> append("&#xA;"); '\t' -> append("&#x9;")
            '\r' -> append("&#xD;"); else -> append(c)
        }
    }
}

fun parseXml(text: String): XmlDocument = XmlParser(text).parseDocument()
