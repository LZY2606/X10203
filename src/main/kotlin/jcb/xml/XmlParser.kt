package jcb.xml

class XmlParseException(message: String, val position: Int) : RuntimeException("$message (位置 $position)")

/**
 * 手写 XML 解析器：不做 DTD/外部实体解析，文本、注释、PI、CDATA 原样保留。
 */
class XmlParser(private val src: String) {
    private var pos = 0
    private val n = src.length

    fun parseDocument(): XmlDocument {
        val prolog = mutableListOf<XmlNode>()
        while (pos < n) {
            if (src.startsWith("<?", pos)) {
                prolog.add(XmlRaw(readPiOrDecl()))
            } else if (src.startsWith("<!--", pos)) {
                prolog.add(XmlRaw(readComment()))
            } else if (src.startsWith("<!", pos)) {
                prolog.add(XmlRaw(readDoctypeOrBogus()))
            } else if (isWhitespaceAt(pos)) {
                prolog.add(XmlText(readWhitespace()))
            } else if (src[pos] == '<') {
                break
            } else {
                throw XmlParseException("根元素之前出现非法文本", pos)
            }
        }
        val root = parseElement()
        val epilog = mutableListOf<XmlNode>()
        while (pos < n) {
            if (isWhitespaceAt(pos)) {
                epilog.add(XmlText(readWhitespace()))
            } else if (src.startsWith("<!--", pos)) {
                epilog.add(XmlRaw(readComment()))
            } else if (src.startsWith("<?", pos)) {
                epilog.add(XmlRaw(readPiOrDecl()))
            } else {
                throw XmlParseException("根元素之后出现非法内容", pos)
            }
        }
        return XmlDocument(prolog, root, epilog)
    }

    private fun parseElement(): XmlElement {
        requireChar('<')
        val tag = readName()
        val el = XmlElement(tag)
        // 属性
        while (true) {
            skipSpaces()
            if (pos >= n) throw XmlParseException("元素 <$tag> 未闭合", pos)
            val c = src[pos]
            if (c == '>') { pos++; break }
            if (c == '/' && pos + 1 < n && src[pos + 1] == '>') { pos += 2; return el }
            val name = readName()
            skipSpaces()
            requireChar('=')
            skipSpaces()
            val value = readAttributeValue()
            el.attributes[name] = value
        }
        // 内容
        val sb = StringBuilder()
        fun flushText() {
            if (sb.isNotEmpty()) { el.children.add(XmlText(sb.toString())); sb.clear() }
        }
        while (pos < n) {
            when {
                src.startsWith("</", pos) -> {
                    flushText()
                    pos += 2
                    val closeName = readName()
                    if (closeName != tag) throw XmlParseException("闭合标签 </$closeName> 与 <$tag> 不匹配", pos)
                    skipSpaces()
                    requireChar('>')
                    return el
                }
                src.startsWith("<!--", pos) -> { flushText(); el.children.add(XmlRaw(readComment())) }
                src.startsWith("<![CDATA[", pos) -> {
                    val end = src.indexOf("]]>", pos + 9)
                    if (end < 0) throw XmlParseException("CDATA 未闭合", pos)
                    sb.append(src, pos, end + 3)
                    pos = end + 3
                }
                src.startsWith("<?", pos) -> { flushText(); el.children.add(XmlRaw(readPiOrDecl())) }
                src.startsWith("<!", pos) -> { flushText(); el.children.add(XmlRaw(readDoctypeOrBogus())) }
                src[pos] == '<' -> { flushText(); el.children.add(parseElement()) }
                else -> { sb.append(src[pos]); pos++ }
            }
        }
        throw XmlParseException("元素 <$tag> 缺少闭合标签", pos)
    }

    private fun readName(): String {
        val start = pos
        if (pos >= n || !(src[pos].isXmlNameStart())) throw XmlParseException("期望 XML 名称", pos)
        while (pos < n && src[pos].isXmlNameChar()) pos++
        return src.substring(start, pos)
    }

    private fun readAttributeValue(): String {
        if (pos >= n || (src[pos] != '"' && src[pos] != '\''))
            throw XmlParseException("属性值缺少引号", pos)
        val quote = src[pos]; pos++
        val start = pos
        while (pos < n && src[pos] != quote) {
            if (src[pos] == '<') throw XmlParseException("属性值中不能出现 <", pos)
            pos++
        }
        if (pos >= n) throw XmlParseException("属性值引号未闭合", start)
        val raw = src.substring(start, pos)
        pos++
        return decodeEntities(raw)
    }

    private fun readComment(): String {
        val start = pos
        val end = src.indexOf("-->", pos + 4)
        if (end < 0) throw XmlParseException("注释未闭合", pos)
        pos = end + 3
        return src.substring(start, pos)
    }

    private fun readPiOrDecl(): String {
        val start = pos
        val end = src.indexOf("?>", pos + 2)
        if (end < 0) throw XmlParseException("处理指令未闭合", pos)
        pos = end + 2
        return src.substring(start, pos)
    }

    private fun readDoctypeOrBogus(): String {
        val start = pos
        var depth = 0
        while (pos < n) {
            val c = src[pos]
            if (c == '[') depth++
            if (c == ']') depth--
            if (c == '>' && depth <= 0) { pos++; return src.substring(start, pos) }
            pos++
        }
        throw XmlParseException("DOCTYPE 未闭合", start)
    }

    private fun skipSpaces() { while (pos < n && src[pos].isWhitespace()) pos++ }
    private fun isWhitespaceAt(p: Int) = p < n && src[p].isWhitespace()
    private fun readWhitespace(): String {
        val start = pos
        while (pos < n && src[pos].isWhitespace()) pos++
        return src.substring(start, pos)
    }
    private fun requireChar(c: Char) {
        if (pos >= n || src[pos] != c) throw XmlParseException("期望 '$c'", pos)
        pos++
    }

    private fun Char.isXmlNameStart(): Boolean =
        this == '_' || this == ':' ||
            this in 'a'..'z' || this in 'A'..'Z' || this.code > 0x7F

    private fun Char.isXmlNameChar(): Boolean =
        isXmlNameStart() || this == '-' || this == '.' || this in '0'..'9' || this.code == 0xB7

    private fun decodeEntities(s: String): String {
        if ('&' !in s) return s
        val out = StringBuilder(s.length)
        var i = 0
        while (i < s.length) {
            val c = s[i]
            if (c != '&') { out.append(c); i++; continue }
            val semi = s.indexOf(';', i + 1)
            if (semi < 0) throw XmlParseException("实体引用未闭合", i)
            val ent = s.substring(i + 1, semi)
            when (ent) {
                "lt" -> out.append('<')
                "gt" -> out.append('>')
                "amp" -> out.append('&')
                "quot" -> out.append('"')
                "apos" -> out.append('\'')
                else -> when {
                    ent.startsWith("#x") || ent.startsWith("#X") ->
                        out.appendCodePoint(ent.substring(2).toInt(16))
                    ent.startsWith("#") ->
                        out.appendCodePoint(ent.substring(1).toInt(10))
                    else -> throw XmlParseException("不支持的实体 &$ent;", i)
                }
            }
            i = semi + 1
        }
        return out.toString()
    }
}
