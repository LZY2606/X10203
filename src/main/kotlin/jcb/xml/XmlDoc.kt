package jcb.xml

/**
 * 保真 XML 文档模型：导入 URDF 时保留全部未知元素、属性、注释、CDATA、
 * 处理指令与原始子节点顺序。序列化结果与原文在文本级别保持一致
 * （未修改部分逐字节往返）。
 */

sealed class XmlContent {
    abstract val raw: String
}

data class XmlText(
    val decoded: String,
    val encoded: String,
) : XmlContent() {
    override val raw: String get() = encoded
}

data class XmlComment(val text: String) : XmlContent() {
    override val raw: String get() = "<!--$text-->"
}

data class XmlCData(val text: String) : XmlContent() {
    override val raw: String get() = "<![CDATA[$text]]>"
}

data class XmlProcessingInstruction(val target: String, val body: String) : XmlContent() {
    override val raw: String get() = "<?$target $body?>"
}

data class XmlAttribute(
    val name: String,
    val decodedValue: String,
    val encodedValue: String,
)

class XmlElement(
    var tag: String,
    val attributes: MutableList<XmlAttribute> = mutableListOf(),
    val children: MutableList<XmlContent> = mutableListOf(),
) : XmlContent() {
    /** 元素自身开闭标签包裹下的原始文本（含子节点）。 */
    var explicitCloseTag: Boolean = true
    var selfClosing: Boolean = false

    fun attr(name: String): String? = attributes.firstOrNull { it.name == name }?.decodedValue

    fun setAttr(name: String, value: String) {
        val idx = attributes.indexOfFirst { it.name == name }
        val encoded = encodeAttribute(value)
        if (idx >= 0) attributes[idx] = XmlAttribute(name, value, encoded)
        else attributes.add(XmlAttribute(name, value, encoded))
    }

    fun childElements(tag: String): List<XmlElement> =
        childElements().filter { it.tag == tag }

    fun childElements(): List<XmlElement> = children.filterIsInstance<XmlElement>()

    fun firstChild(tag: String): XmlElement? = childElements(tag).firstOrNull()

    fun textTrimmed(): String =
        children.filterIsInstance<XmlText>().joinToString("") { it.decoded }.trim()

    override val raw: String get() = serialize(this)
}

data class XmlDocument(
    val declaration: String?,
    val content: MutableList<XmlContent>,
) {
    fun root(): XmlElement = content.filterIsInstance<XmlElement>().first()
    fun serialize(): String = buildString {
        if (declaration != null) {
            append(declaration)
            if (!declaration.endsWith("\n")) append("\n")
        }
        content.forEach { append(it.raw) }
    }
}

internal fun encodeAttribute(value: String): String = buildString {
    value.forEach { c ->
        when (c) {
            '<' -> append("&lt;")
            '>' -> append("&gt;")
            '&' -> append("&amp;")
            '"' -> append("&quot;")
            '\'' -> append("&apos;")
            '\n' -> append("&#10;")
            '\r' -> append("&#13;")
            '\t' -> append("&#9;")
            else -> append(c)
        }
    }
}

internal fun encodeText(value: String): String = buildString {
    value.forEach { c ->
        when (c) {
            '<' -> append("&lt;")
            '>' -> append("&gt;")
            '&' -> append("&amp;")
            else -> append(c)
        }
    }
}

fun serialize(element: XmlElement): String = buildString {
    append('<').append(element.tag)
    element.attributes.forEach { a ->
        append(' ').append(a.name).append("=\"").append(a.encodedValue).append('"')
    }
    val hasContent = element.children.isNotEmpty()
    if (!hasContent && element.selfClosing) {
        append("/>")
        return@buildString
    }
    append('>')
    element.children.forEach { append(it.raw) }
    if (hasContent || element.explicitCloseTag) {
        append("</").append(element.tag).append('>')
    }
}

private class XmlParseException(message: String, val position: Int) : RuntimeException("$message (at $position)")

/**
 * 手写保真解析器：不做任何规范化，文本/属性同时保存编码前后两种形态，
 * 因此修改语义值后再导出时，未触碰的内容与原文完全一致。
 */
class XmlParser(private val text: String) {
    private var pos = 0

    companion object {
        fun parse(text: String): XmlDocument = XmlParser(text).parseDocument()
    }

    private fun peek(): Char? = text.getOrNull(pos)
    private fun peekAt(offset: Int): Char? = text.getOrNull(pos + offset)

    private fun parseDocument(): XmlDocument {
        var declaration: String? = null
        val content = mutableListOf<XmlContent>()
        skipMiscSpace()
        if (text.startsWith("<?xml", pos)) {
            val end = text.indexOf("?>", pos)
                ?: throw XmlParseException("未闭合的 XML 声明", pos)
            declaration = text.substring(pos, end + 2)
            pos = end + 2
        }
        while (pos < text.length) {
            val node = parseNode() ?: break
            content.add(node)
        }
        return XmlDocument(declaration, content)
    }

    private fun skipMiscSpace() {
        while (pos < text.length && text[pos].isWhitespace()) pos++
    }

    private fun parseNode(): XmlContent? {
        skipMiscSpace()
        if (pos >= text.length) return null
        if (text.startsWith("<!--", pos)) return parseComment()
        if (text.startsWith("<![CDATA[", pos)) return parseCData()
        if (text.startsWith("<?", pos)) return parsePi()
        if (text.startsWith("</", pos)) return null
        if (peek() == '<') return parseElement()
        return parseText()
    }

    private fun parseComment(): XmlComment {
        val start = pos + 4
        val end = text.indexOf("-->", start)
            ?: throw XmlParseException("未闭合的注释", pos)
        val body = text.substring(start, end)
        pos = end + 3
        return XmlComment(body)
    }

    private fun parseCData(): XmlCData {
        val start = pos + 9
        val end = text.indexOf("]]>", start)
            ?: throw XmlParseException("未闭合的 CDATA", pos)
        val body = text.substring(start, end)
        pos = end + 3
        return XmlCData(body)
    }

    private fun parsePi(): XmlProcessingInstruction {
        val start = pos + 2
        val end = text.indexOf("?>", start)
            ?: throw XmlParseException("未闭合的处理指令", pos)
        val body = text.substring(start, end)
        pos = end + 2
        val firstSpace = body.indexOf(' ')
        return if (firstSpace < 0) XmlProcessingInstruction(body.trim(), "")
        else XmlProcessingInstruction(body.substring(0, firstSpace), body.substring(firstSpace + 1))
    }

    private fun parseText(): XmlText {
        val start = pos
        while (pos < text.length && text[pos] != '<') pos++
        val encoded = text.substring(start, pos)
        return XmlText(decodeEntities(encoded, start), encoded)
    }

    private fun parseElement(): XmlElement {
        val startPos = pos
        pos++ // 消费 '<'
        val tag = parseName()
        val attrs = mutableListOf<XmlAttribute>()
        while (true) {
            skipWhitespace()
            if (pos >= text.length) throw XmlParseException("未闭合的标签 <$tag>", startPos)
            val c = text[pos]
            if (c == '/' && text[pos + 1] == '>') {
                pos += 2
                return XmlElement(tag, attrs, mutableListOf()).also { it.selfClosing = true }
            }
            if (c == '>') {
                pos++
                break
            }
            val attr = parseAttribute()
            attrs.add(attr)
        }
        val children = mutableListOf<XmlContent>()
        while (true) {
            if (pos >= text.length) throw XmlParseException("缺少 </$tag>", startPos)
            if (text.startsWith("</", pos)) {
                pos += 2
                val closeName = parseName()
                if (closeName != tag) throw XmlParseException("闭合标签 </$closeName> 与 <$tag> 不匹配", pos)
                skipWhitespace()
                if (peek() != '>') throw XmlParseException("闭合标签语法错误", pos)
                pos++
                return XmlElement(tag, attrs, children)
            }
            val node = parseNode()
                ?: throw XmlParseException("元素 <$tag> 内部结构错误", pos)
            children.add(node)
        }
    }

    private fun parseName(): String {
        val start = pos
        while (pos < text.length) {
            val c = text[pos]
            if (c.isWhitespace() || c == '=' || c == '>' || c == '/' || c == '<') break
            pos++
        }
        if (start == pos) throw XmlParseException("期望 XML 名称", pos)
        return text.substring(start, pos)
    }

    private fun skipWhitespace() {
        while (pos < text.length && text[pos].isWhitespace()) pos++
    }

    private fun parseAttribute(): XmlAttribute {
        val name = parseName()
        skipWhitespace()
        if (peek() != '=') throw XmlParseException("属性 $name 后缺少 =", pos)
        pos++
        skipWhitespace()
        val quote = peek()
        if (quote != '"' && quote != '\'') throw XmlParseException("属性 $name 的值缺少引号", pos)
        pos++
        val start = pos
        while (pos < text.length && text[pos] != quote) pos++
        if (pos >= text.length) throw XmlParseException("属性 $name 的值未闭合", start)
        val encoded = text.substring(start, pos)
        pos++ // 消费引号
        return XmlAttribute(name, decodeEntities(encoded, start), encoded)
    }

    private fun decodeEntities(value: String, at: Int): String = buildString {
        var i = 0
        while (i < value.length) {
            val c = value[i]
            if (c == '&') {
                val semi = value.indexOf(';', i + 1)
                if (semi < 0) throw XmlParseException("非法实体引用", at + i)
                val entity = value.substring(i + 1, semi)
                when (entity) {
                    "lt" -> append('<')
                    "gt" -> append('>')
                    "amp" -> append('&')
                    "quot" -> append('"')
                    "apos" -> append('\'')
                    else -> when {
                        entity.startsWith("#x") || entity.startsWith("#X") ->
                            appendCode(entity.substring(2).toIntOrNull(16), at + i)
                        entity.startsWith("#") ->
                            appendCode(entity.substring(1).toIntOrNull(), at + i)
                        else -> {
                            // 未知实体（如 &foo;）原样保留，保证往返一致
                            append('&').append(entity).append(';')
                        }
                    }
                }
                i = semi + 1
            } else {
                append(c)
                i++
            }
        }
    }

    private fun StringBuilder.appendCode(code: Int?, position: Int) {
        if (code == null) throw XmlParseException("非法字符引用", position)
        appendCodePoint(code)
    }

    private fun StringBuilder.appendCodePoint(code: Int) {
        if (code < 0x10000) {
            append(code.toChar())
        } else {
            val adjusted = code - 0x10000
            append((0xD800 + (adjusted ushr 10)).toChar())
            append((0xDC00 + (adjusted and 0x3FF)).toChar())
        }
    }
}
