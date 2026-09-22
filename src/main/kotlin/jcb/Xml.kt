package jcb

/** Minimal XML DOM that preserves unknown elements, attributes, comments and
 *  original document order so unrecognized content round-trips byte-faithfully. */
sealed interface XNode {
    fun render(sb: StringBuilder)
}

data class XText(val text: String) : XNode {
    override fun render(sb: StringBuilder) {
        sb.append(text)
    }
}

data class XComment(val text: String) : XNode {
    override fun render(sb: StringBuilder) {
        sb.append("<!--").append(text).append("-->")
    }
}

data class XElement(
    val name: String,
    val attrs: LinkedHashMap<String, String> = LinkedHashMap(),
    val children: MutableList<XNode> = mutableListOf(),
    var selfClosing: Boolean = false,
) : XNode {
    fun attr(name: String): String? = attrs[name]
    fun elements(name: String): List<XElement> = children.filterIsInstance<XElement>().filter { it.name == name }
    fun firstElement(name: String): XElement? = elements(name).firstOrNull()
    fun text(): String = children.filterIsInstance<XText>().joinToString("") { it.text }.trim()

    override fun render(sb: StringBuilder) {
        sb.append('<').append(name)
        for ((k, v) in attrs) {
            sb.append(' ').append(k).append("=\"").append(escapeAttr(v)).append('"')
        }
        if (children.isEmpty() && selfClosing) {
            sb.append("/>")
        } else {
            sb.append('>')
            for (c in children) c.render(sb)
            sb.append("</").append(name).append('>')
        }
    }

    companion object {
        fun escapeAttr(s: String): String = s
            .replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
            .replace("\"", "&quot;")
        fun escapeText(s: String): String = s
            .replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
    }
}

class XDocument(val prolog: String, val root: XElement, val trailing: MutableList<XNode> = mutableListOf()) {
    fun render(): String {
        val sb = StringBuilder()
        sb.append(prolog)
        root.render(sb)
        for (n in trailing) n.render(sb)
        return sb.toString()
    }
}

object XmlParser {
    fun parse(input: String): XDocument {
        val p = Parser(input)
        return p.parseDocument()
    }

    private class Parser(val s: String) {
        var i = 0
        fun parseDocument(): XDocument {
            val sb = StringBuilder()
            // prolog: everything up to the first element start that is not <?...?> or <!--...--> or doctype
            while (i < s.length) {
                if (s.startsWith("<?", i)) {
                    val end = s.indexOf("?>", i)
                    sb.append(s.substring(i, end + 2)); i = end + 2
                } else if (s.startsWith("<!--", i)) {
                    val end = s.indexOf("-->", i)
                    sb.append(s.substring(i, end + 3)); i = end + 3
                } else if (s.startsWith("<!", i)) {
                    val end = s.indexOf('>', i)
                    sb.append(s.substring(i, end + 1)); i = end + 1
                } else if (s[i].isWhitespace()) {
                    sb.append(s[i]); i++
                } else break
            }
            val root = parseElement()
            val trailing = mutableListOf<XNode>()
            while (i < s.length) {
                if (s.startsWith("<!--", i)) {
                    val end = s.indexOf("-->", i)
                    trailing.add(XComment(s.substring(i + 4, end))); i = end + 3
                } else {
                    val start = i
                    while (i < s.length && !s.startsWith("<!--", i)) i++
                    val t = s.substring(start, i)
                    if (t.isNotEmpty()) trailing.add(XText(t))
                }
            }
            return XDocument(sb.toString(), root, trailing)
        }

        fun parseElement(): XElement {
            expect('<')
            val name = readName()
            val attrs = LinkedHashMap<String, String>()
            while (true) {
                skipWs()
                if (i >= s.length) error("unexpected EOF in element $name")
                if (s.startsWith("/>", i)) {
                    i += 2
                    return XElement(name, attrs, mutableListOf(), selfClosing = true)
                }
                if (s[i] == '>') {
                    i++
                    break
                }
                val an = readName()
                skipWs(); expect('='); skipWs()
                val q = s[i]; i++
                val start = i
                while (i < s.length && s[i] != q) i++
                attrs[an] = unescape(s.substring(start, i))
                i++
            }
            val children = mutableListOf<XNode>()
            while (true) {
                if (i >= s.length) error("unexpected EOF, expected </$name>")
                if (s.startsWith("</", i)) {
                    val end = s.indexOf('>', i)
                    i = end + 1
                    break
                } else if (s.startsWith("<!--", i)) {
                    val end = s.indexOf("-->", i)
                    children.add(XComment(s.substring(i + 4, end))); i = end + 3
                } else if (s.startsWith("<![CDATA[", i)) {
                    val end = s.indexOf("]]>", i)
                    children.add(XText(s.substring(i, end + 3))); i = end + 3
                } else if (s[i] == '<') {
                    children.add(parseElement())
                } else {
                    val start = i
                    while (i < s.length && s[i] != '<') i++
                    children.add(XText(s.substring(start, i)))
                }
            }
            return XElement(name, attrs, children)
        }

        fun readName(): String {
            val start = i
            while (i < s.length && (s[i].isLetterOrDigit() || s[i] in "._:-")) i++
            return s.substring(start, i)
        }
        fun skipWs() { while (i < s.length && s[i].isWhitespace()) i++ }
        fun expect(c: Char) { if (s[i] != c) error("expected '$c' at $i in '$s'"); i++ }
        fun unescape(v: String): String = v
            .replace("&quot;", "\"").replace("&apos;", "'")
            .replace("&lt;", "<").replace("&gt;", ">").replace("&amp;", "&")
    }
}
