package atlas

sealed interface XNode

data class XText(val text: String) : XNode

data class XComment(val text: String) : XNode

class XElement(val name: String) : XNode {
    val attrs = LinkedHashMap<String, String>()
    val children = mutableListOf<XNode>()

    fun attr(n: String): String? = attrs[n]
    fun child(n: String): XElement? = children.filterIsInstance<XElement>().firstOrNull { it.name == n }
    fun children(n: String): List<XElement> = children.filterIsInstance<XElement>().filter { it.name == n }
    fun text(): String = children.filterIsInstance<XText>().joinToString("") { it.text }.trim()
}

/**
 * Minimal XML DOM that preserves unknown elements, attributes, comments,
 * text nodes and their original order so documents round-trip on export.
 */
object Xml {
    fun parse(src: String): XElement {
        val doc = XElement("#document")
        val stack = ArrayDeque<XElement>()
        stack.addLast(doc)
        var i = 0
        val n = src.length
        while (i < n) {
            if (src[i] == '<') {
                when {
                    src.startsWith("<!--", i) -> {
                        val end = src.indexOf("-->", i).let { if (it < 0) n - 3 else it }
                        stack.last().children.add(XComment(src.substring(i + 4, end)))
                        i = end + 3
                    }
                    src.startsWith("<?", i) -> {
                        val end = src.indexOf("?>", i).let { if (it < 0) n - 2 else it }
                        i = end + 2
                    }
                    src.startsWith("<!", i) -> {
                        val end = src.indexOf('>', i).let { if (it < 0) n - 1 else it }
                        i = end + 1
                    }
                    src.startsWith("</", i) -> {
                        val end = src.indexOf('>', i)
                        if (stack.size > 1) stack.removeLast()
                        i = if (end < 0) n else end + 1
                    }
                    else -> {
                        var j = i + 1
                        while (j < n && isNameChar(src[j])) j++
                        val el = XElement(src.substring(i + 1, j))
                        var selfClosing = false
                        while (j < n) {
                            while (j < n && src[j].isWhitespace()) j++
                            if (j >= n) break
                            if (src[j] == '/') { selfClosing = true; j++; continue }
                            if (src[j] == '>') break
                            var k = j
                            while (k < n && isNameChar(src[k])) k++
                            val aname = src.substring(j, k)
                            while (k < n && src[k].isWhitespace()) k++
                            if (k < n && src[k] == '=') {
                                k++
                                while (k < n && src[k].isWhitespace()) k++
                                if (k < n && (src[k] == '"' || src[k] == '\'')) {
                                    val q = src[k]
                                    val vstart = k + 1
                                    val vend = src.indexOf(q, vstart).let { if (it < 0) n else it }
                                    el.attrs[aname] = decode(src.substring(vstart, vend))
                                    k = vend + 1
                                }
                            } else {
                                el.attrs[aname] = ""
                            }
                            j = k
                        }
                        i = if (j < n) j + 1 else n
                        stack.last().children.add(el)
                        if (!selfClosing) stack.addLast(el)
                    }
                }
            } else {
                var end = src.indexOf('<', i)
                if (end < 0) end = n
                stack.last().children.add(XText(decode(src.substring(i, end))))
                i = end
            }
        }
        return doc
    }

    fun document(doc: XElement): XElement =
        doc.children.filterIsInstance<XElement>().firstOrNull()
            ?: throw IllegalArgumentException("XML 中没有根元素")

    fun serialize(doc: XElement): String {
        val sb = StringBuilder()
        sb.append("<?xml version=\"1.0\"?>\n")
        for (node in doc.children) write(node, sb)
        return sb.toString()
    }

    private fun write(node: XNode, sb: StringBuilder) {
        when (node) {
            is XText -> sb.append(escape(node.text))
            is XComment -> sb.append("<!--").append(node.text).append("-->")
            is XElement -> {
                sb.append('<').append(node.name)
                for ((k, v) in node.attrs) {
                    sb.append(' ').append(k).append("=\"").append(escapeAttr(v)).append('"')
                }
                if (node.children.isEmpty()) {
                    sb.append("/>")
                } else {
                    sb.append('>')
                    for (ch in node.children) write(ch, sb)
                    sb.append("</").append(node.name).append('>')
                }
            }
        }
    }

    private fun isNameChar(c: Char) = c.isLetterOrDigit() || c == '_' || c == '-' || c == ':' || c == '.'

    fun decode(s: String): String = s
        .replace("&lt;", "<").replace("&gt;", ">")
        .replace("&quot;", "\"").replace("&apos;", "'")
        .replace("&amp;", "&")

    fun escape(s: String): String = s
        .replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")

    fun escapeAttr(s: String): String = escape(s).replace("\"", "&quot;")
}
