package coordbook

/**
 * 通用 XML DOM：导入时保留未知元素、未知属性、原始子节点顺序、文本、注释与 CDATA。
 * 不做 schema 感知，URDF 解析器在其上读取已知节点，写回时未识别内容原样往返。
 */
sealed interface XmlContent {
    fun write(sb: StringBuilder)
}

class XmlText(val raw: String) : XmlContent {
    override fun write(sb: StringBuilder) {
        sb.append(raw)
    }
}

class XmlCData(val raw: String) : XmlContent {
    override fun write(sb: StringBuilder) {
        sb.append("<![CDATA[").append(raw).append("]]>")
    }
}

class XmlComment(val raw: String) : XmlContent {
    override fun write(sb: StringBuilder) {
        sb.append("<!--").append(raw).append("-->")
    }
}

class XmlProcessingInstruction(val raw: String) : XmlContent {
    override fun write(sb: StringBuilder) {
        sb.append("<?").append(raw).append("?>")
    }
}

class XmlElement(
    var tag: String,
    val attributes: MutableList<Triple<String, String, Char>> = mutableListOf(),
    val content: MutableList<XmlContent> = mutableListOf(),
) : XmlContent {

    /** 属性值已做实体反转义。 */
    private fun rawAttr(name: String): String? =
        attributes.firstOrNull { it.first == name }?.second

    fun child(name: String): XmlElement? = children(name).firstOrNull()

    fun children(name: String): List<XmlElement> =
        content.filterIsInstance<XmlElement>().filter { it.tag == name }

    fun allElements(): List<XmlElement> = content.filterIsInstance<XmlElement>()

    fun attr(name: String): String? =
        attributes.firstOrNull { it.first == name }?.let { XmlParser.unescape(it.second) }

    fun attrRaw(name: String): String? = rawAttr(name)

    fun requireAttr(name: String): String =
        attr(name) ?: throw XmlException("元素 <$tag> 缺少属性 \"$name\"")

    fun doubleAttr(name: String): Double? = attr(name)?.let {
        it.trim().toDoubleOrNull() ?: throw XmlException("<$tag $name=\"$it\"> 不是合法数字")
    }

    /** 设置属性；已存在则原地改值（保持顺序），不存在则追加。 */
    fun setAttr(name: String, value: String) {
        val idx = attributes.indexOfFirst { it.first == name }
        if (idx >= 0) {
            val old = attributes[idx]
            attributes[idx] = Triple(name, XmlElement.escapeFor(value, old.third), old.third)
        } else {
            attributes.add(Triple(name, XmlElement.escapeFor(value, '"'), '"'))
        }
    }

    fun textOrNull(): String? {
        val parts = content.filterIsInstance<XmlText>().map { it.raw }
        return if (parts.isEmpty()) null else parts.joinToString("").trim()
    }

    override fun write(sb: StringBuilder) {
        sb.append('<').append(tag)
        for ((k, raw, quote) in attributes) {
            sb.append(' ').append(k).append('=').append(quote).append(raw).append(quote)
        }
        val meaningful = content.any { it !is XmlText || it.raw.isNotBlank() }
        if (!meaningful) {
            sb.append("/>")
            return
        }
        sb.append('>')
        for (node in content) node.write(sb)
        sb.append("</").append(tag).append('>')
    }

    fun toXmlString(declaration: String?): String {
        val sb = StringBuilder()
        if (declaration != null) sb.append("<?").append(declaration).append("?>\n")
        write(sb)
        sb.append('\n')
        return sb.toString()
    }

    companion object {
        /** 按该属性实际使用的引号转义：只处理必须处理的字符，保持往返稳定。 */
        fun escapeFor(s: String, quote: Char): String = buildString(s.length) {
            for (ch in s) when (ch) {
                '<' -> append("&lt;")
                '>' -> append("&gt;")
                '&' -> append("&amp;")
                quote -> if (quote == '"') append("&quot;") else append("&apos;")
                else -> append(ch)
            }
        }

        fun escapeAttr(s: String): String = escapeFor(s, '"')

        fun escapeText(s: String): String = buildString(s.length) {
            for (ch in s) when (ch) {
                '<' -> append("&lt;")
                '>' -> append("&gt;")
                '&' -> append("&amp;")
                else -> append(ch)
            }
        }
    }
}

class XmlException(message: String) : RuntimeException(message)

object XmlParser {

    private class Cursor(val s: String, var pos: Int = 0)

    fun parse(input: String): Pair<XmlElement, String?> {
        val c = Cursor(input)
        var declaration: String? = null
        while (true) {
            skipInsignificant(c)
            when {
                c.s.startsWith("<?", c.pos) -> {
                    val pi = readProcessingInstruction(c)
                    if (declaration == null && pi.trimStart().startsWith("xml")) declaration = pi
                }
                c.s.startsWith("<!--", c.pos) -> readComment(c)
                c.s.startsWith("<!", c.pos) -> readBang(c)
                else -> break
            }
        }
        val root = readElement(c)
            ?: throw XmlException("未找到 XML 根元素")
        return root to declaration
    }

    private fun skipInsignificant(c: Cursor) {
        while (c.pos < c.s.length && c.s[c.pos].isWhitespace()) c.pos++
    }

    private fun readProcessingInstruction(c: Cursor): String {
        // 假定 c.pos 指向 "<?"
        c.pos += 2
        val end = c.s.indexOf("?>", c.pos)
        if (end < 0) throw XmlException("处理指令缺少 ?> 结束")
        val raw = c.s.substring(c.pos, end)
        c.pos = end + 2
        return raw
    }

    private fun readComment(c: Cursor): String {
        val end = c.s.indexOf("-->", c.pos + 4)
        if (end < 0) throw XmlException("注释缺少 --> 结束")
        val raw = c.s.substring(c.pos + 4, end)
        c.pos = end + 3
        return raw
    }

    private fun readBang(c: Cursor) {
        // DOCTYPE 等声明，读到匹配的 '>'
        val end = c.s.indexOf('>', c.pos)
        if (end < 0) throw XmlException("声明缺少 > 结束")
        c.pos = end + 1
    }

    private fun readElement(c: Cursor): XmlElement? {
        skipInsignificant(c)
        if (!c.s.startsWith("<", c.pos) || c.s.startsWith("<?", c.pos) ||
            c.s.startsWith("<!--", c.pos) || c.s.startsWith("<!", c.pos)
        ) return null
        c.pos++ // 消费 '<'
        val nameStart = c.pos
        while (c.pos < c.s.length && !c.s[c.pos].isWhitespace() &&
            c.s[c.pos] != '>' && c.s[c.pos] != '/'
        ) c.pos++
        val tag = c.s.substring(nameStart, c.pos)
        val el = XmlElement(tag)
        parseAttributes(c, el)
        // parseAttributes 消费到 '>' 或 '/>'
        return el
    }

    private fun parseAttributes(c: Cursor, el: XmlElement) {
        while (true) {
            while (c.pos < c.s.length && c.s[c.pos].isWhitespace()) c.pos++
            if (c.s.startsWith("/>", c.pos)) {
                c.pos += 2
                return
            }
            if (c.s.startsWith(">", c.pos)) {
                c.pos++
                readContent(c, el)
                return
            }
            val nameStart = c.pos
            while (c.pos < c.s.length && c.s[c.pos] != '=' &&
                !c.s[c.pos].isWhitespace() && c.s[c.pos] != '>' && c.s[c.pos] != '/'
            ) c.pos++
            val name = c.s.substring(nameStart, c.pos)
            while (c.pos < c.s.length && c.s[c.pos].isWhitespace()) c.pos++
            if (c.pos >= c.s.length || c.s[c.pos] != '=') {
                // 无值属性：URDF 不常见，但保留为布尔属性
                el.attributes.add(Triple(name, "", '"'))
                continue
            }
            c.pos++ // '='
            while (c.pos < c.s.length && c.s[c.pos].isWhitespace()) c.pos++
            val quote = c.s[c.pos]
            if (quote != '"' && quote != '\'')
                throw XmlException("属性 $name 的值缺少引号（位置 ${c.pos}）")
            c.pos++
            val valueStart = c.pos
            while (c.pos < c.s.length && c.s[c.pos] != quote) c.pos++
            if (c.pos >= c.s.length) throw XmlException("属性 $name 的值未闭合")
            val rawValue = c.s.substring(valueStart, c.pos)
            c.pos++ // 消费引号
            el.attributes.add(Triple(name, rawValue, quote))
        }
    }

    private fun readContent(c: Cursor, parent: XmlElement) {
        val text = StringBuilder()
        while (c.pos < c.s.length) {
            when {
                c.s.startsWith("</", c.pos) -> {
                    c.pos += 2
                    val end = c.s.indexOf('>', c.pos)
                    if (end < 0) throw XmlException("结束标签缺少 >")
                    val closeTag = c.s.substring(c.pos, end).trim()
                    if (closeTag != parent.tag)
                        throw XmlException("标签不匹配：<${parent.tag}> 由 </$closeTag> 关闭")
                    c.pos = end + 1
                    if (text.isNotEmpty()) parent.content.add(XmlText(text.toString()))
                    return
                }
                c.s.startsWith("<!--", c.pos) -> {
                    if (text.isNotEmpty()) {
                        parent.content.add(XmlText(text.toString()))
                        text.setLength(0)
                    }
                    parent.content.add(XmlComment(readComment(c)))
                }
                c.s.startsWith("<![CDATA[", c.pos) -> {
                    if (text.isNotEmpty()) {
                        parent.content.add(XmlText(text.toString()))
                        text.setLength(0)
                    }
                    val start = c.pos + 9
                    val end = c.s.indexOf("]]>", start)
                    if (end < 0) throw XmlException("CDATA 未闭合")
                    parent.content.add(XmlCData(c.s.substring(start, end)))
                    c.pos = end + 3
                }
                c.s.startsWith("<?", c.pos) -> {
                    if (text.isNotEmpty()) {
                        parent.content.add(XmlText(text.toString()))
                        text.setLength(0)
                    }
                    parent.content.add(XmlProcessingInstruction(readProcessingInstruction(c)))
                }
                c.s.startsWith("<", c.pos) -> {
                    if (text.isNotEmpty()) {
                        parent.content.add(XmlText(text.toString()))
                        text.setLength(0)
                    }
                    val child = readElement(c)
                        ?: throw XmlException("位置 ${c.pos} 处无法解析元素")
                    parent.content.add(child)
                }
                else -> {
                    text.append(c.s[c.pos])
                    c.pos++
                }
            }
        }
        throw XmlException("元素 <${parent.tag}> 缺少结束标签")
    }

    fun escapeRaw(s: String, forAttribute: Boolean): String =
        if (forAttribute) XmlElement.escapeAttr(s) else XmlElement.escapeText(s)

    private const val ENTITIES = "\"'&<>"

    fun unescape(s: String): String {
        if ('&' !in s) return s
        val sb = StringBuilder(s.length)
        var i = 0
        while (i < s.length) {
            if (s[i] != '&') {
                sb.append(s[i]); i++; continue
            }
            val semi = s.indexOf(';', i + 1)
            if (semi < 0 || semi - i > 12) {
                sb.append(s[i]); i++; continue
            }
            val ent = s.substring(i + 1, semi)
            when (ent) {
                "lt" -> sb.append('<')
                "gt" -> sb.append('>')
                "amp" -> sb.append('&')
                "apos" -> sb.append('\'')
                "quot" -> sb.append('"')
                else -> if (ent.startsWith("#x") || ent.startsWith("#X")) {
                    sb.appendCodePoint(ent.substring(2).toInt(16))
                } else if (ent.startsWith("#")) {
                    sb.appendCodePoint(ent.substring(1).toInt())
                } else {
                    sb.append('&').append(ent).append(';')
                }
            }
            i = semi + 1
        }
        return sb.toString()
    }

}
