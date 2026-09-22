package atlas.xml

/**
 * Minimal XML model that preserves unknown elements, unknown attributes,
 * attribute order and child order so unrecognized content round-trips.
 */
sealed interface XmlNode {
    fun serialize(out: StringBuilder)
}

class XmlText(val text: String) : XmlNode {
    override fun serialize(out: StringBuilder) {
        out.append(escapeText(text))
    }
}

class XmlComment(val text: String) : XmlNode {
    override fun serialize(out: StringBuilder) {
        out.append("<!--").append(text).append("-->")
    }
}

class XmlElement(
    var name: String,
    val attributes: LinkedHashMap<String, String> = linkedMapOf(),
    val children: MutableList<XmlNode> = mutableListOf()
) : XmlNode {
    fun attr(name: String): String? = attributes[name]

    fun childElements(name: String? = null): List<XmlElement> =
        children.filterIsInstance<XmlElement>().filter { name == null || it.name == name }

    fun firstChild(name: String): XmlElement? = childElements(name).firstOrNull()

    fun text(): String = children.filterIsInstance<XmlText>().joinToString("") { it.text }.trim()

    override fun serialize(out: StringBuilder) {
        out.append('<').append(name)
        for ((k, v) in attributes) {
            out.append(' ').append(k).append("=\"").append(escapeAttr(v)).append('"')
        }
        if (children.isEmpty()) {
            out.append("/>")
        } else {
            out.append('>')
            for (c in children) c.serialize(out)
            out.append("</").append(name).append('>')
        }
    }

    fun deepCopy(): XmlElement {
        val copy = XmlElement(name, LinkedHashMap(attributes))
        for (c in children) {
            copy.children.add(
                when (c) {
                    is XmlElement -> c.deepCopy()
                    is XmlText -> XmlText(c.text)
                    is XmlComment -> XmlComment(c.text)
                }
            )
        }
        return copy
    }
}

class XmlDocument(
    var prolog: String = "<?xml version=\"1.0\"?>",
    var root: XmlElement
) {
    fun serialize(): String {
        val sb = StringBuilder()
        if (prolog.isNotEmpty()) sb.append(prolog).append('\n')
        root.serialize(sb)
        sb.append('\n')
        return sb.toString()
    }
}

fun escapeText(s: String): String =
    s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")

fun escapeAttr(s: String): String =
    s.replace("&", "&amp;").replace("<", "&lt;").replace("\"", "&quot;")

private fun unescape(s: String): String =
    s.replace("&lt;", "<").replace("&gt;", ">").replace("&quot;", "\"")
        .replace("&apos;", "'").replace("&amp;", "&")

object XmlParser {
    fun parse(input: String): XmlDocument {
        val p = Parser(input)
        return p.parseDocument()
    }

    private class Parser(val s: String) {
        var pos = 0

        fun parseDocument(): XmlDocument {
            var prolog = ""
            var root: XmlElement? = null
            while (pos < s.length) {
                when {
                    s.startsWith("<?", pos) -> {
                        val end = s.indexOf("?>", pos)
                        require(end >= 0) { "Unterminated prolog" }
                        prolog = s.substring(pos, end + 2)
                        pos = end + 2
                    }
                    s.startsWith("<!--", pos) -> skipComment()
                    s.startsWith("<!DOCTYPE", pos) || s.startsWith("<!", pos) -> skipDecl()
                    s.startsWith("<", pos) -> {
                        root = parseElement()
                        // ignore trailing whitespace
                        while (pos < s.length && s[pos].isWhitespace()) pos++
                        require(pos >= s.length) { "Trailing content after root element" }
                        return XmlDocument(prolog, root)
                    }
                    s[pos].isWhitespace() -> pos++
                    else -> throw IllegalArgumentException("Unexpected content at $pos")
                }
            }
            throw IllegalArgumentException("No root element")
        }

        private fun skipComment() {
            val end = s.indexOf("-->", pos)
            require(end >= 0) { "Unterminated comment" }
            pos = end + 3
        }

        private fun skipDecl() {
            val end = s.indexOf('>', pos)
            require(end >= 0) { "Unterminated declaration" }
            pos = end + 1
        }

        fun parseElement(): XmlElement {
            require(s[pos] == '<')
            pos++
            val name = readName()
            val el = XmlElement(name)
            // attributes
            while (true) {
                skipWs()
                when {
                    pos >= s.length -> throw IllegalArgumentException("Unexpected EOF in tag <$name>")
                    s.startsWith("/>", pos) -> {
                        pos += 2
                        return el
                    }
                    s[pos] == '>' -> {
                        pos++
                        parseChildren(el)
                        return el
                    }
                    else -> {
                        val aname = readName()
                        skipWs()
                        require(s[pos] == '=') { "Expected '=' after attribute $aname" }
                        pos++
                        skipWs()
                        val quote = s[pos]
                        require(quote == '"' || quote == '\'') { "Attribute $aname must be quoted" }
                        pos++
                        val start = pos
                        while (pos < s.length && s[pos] != quote) pos++
                        el.attributes[aname] = unescape(s.substring(start, pos))
                        pos++
                    }
                }
            }
        }

        private fun parseChildren(el: XmlElement) {
            val textBuf = StringBuilder()
            fun flushText() {
                if (textBuf.isNotEmpty()) {
                    el.children.add(XmlText(textBuf.toString()))
                    textBuf.clear()
                }
            }
            while (pos < s.length) {
                when {
                    s.startsWith("</", pos) -> {
                        flushText()
                        pos += 2
                        val close = readName()
                        require(close == el.name) { "Mismatched close tag: expected ${el.name}, got $close" }
                        skipWs()
                        require(s[pos] == '>') { "Expected '>'" }
                        pos++
                        return
                    }
                    s.startsWith("<!--", pos) -> {
                        flushText()
                        val end = s.indexOf("-->", pos)
                        require(end >= 0) { "Unterminated comment" }
                        el.children.add(XmlComment(s.substring(pos + 4, end)))
                        pos = end + 3
                    }
                    s.startsWith("<![CDATA[", pos) -> {
                        val end = s.indexOf("]]>", pos)
                        require(end >= 0) { "Unterminated CDATA" }
                        textBuf.append(s.substring(pos + 9, end))
                        pos = end + 3
                    }
                    s.startsWith("<!", pos) -> {
                        flushText()
                        skipDecl()
                    }
                    s.startsWith("<?", pos) -> {
                        flushText()
                        val end = s.indexOf("?>", pos)
                        require(end >= 0) { "Unterminated PI" }
                        pos = end + 2
                    }
                    s[pos] == '<' -> {
                        flushText()
                        el.children.add(parseElement())
                    }
                    else -> textBuf.append(s[pos++])
                }
            }
            throw IllegalArgumentException("Unexpected EOF inside <${el.name}>")
        }

        private fun readName(): String {
            val start = pos
            while (pos < s.length && (s[pos].isLetterOrDigit() || s[pos] in "._:-")) pos++
            require(pos > start) { "Expected name at $start" }
            return s.substring(start, pos)
        }

        private fun skipWs() {
            while (pos < s.length && s[pos].isWhitespace()) pos++
        }
    }
}
