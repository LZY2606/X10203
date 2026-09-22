package book.xml

/**
 * 基于原始文本的 XML 解析器。不使用 DOM/SAX，因此：
 *  - 未知元素与属性原样保留
 *  - 注释、处理指令、DOCTYPE、XML 声明都作为 RawText 保留
 *  - 子节点顺序与属性顺序保持文档顺序
 */
class RawXmlParser(private val text: String) {
    private var pos = 0

    fun parse(): RawDocument {
        val doc = RawDocument()
        var pending = StringBuilder()
        val stack = ArrayDeque<RawElement>()

        fun flushText() {
            if (pending.isNotEmpty()) {
                val node = RawText(pending.toString())
                if (stack.isEmpty()) doc.prolog.add(node) else stack.last().children.add(node)
                pending = StringBuilder()
            }
        }

        while (pos < text.length) {
            if (text[pos] != '<') {
                pending.append(text[pos])
                pos++
                continue
            }
            when {
                text.startsWith("<!--", pos) -> {
                    flushText()
                    val end = text.indexOf("-->", pos + 4)
                    val endPos = if (end < 0) text.length else end + 3
                    val node = RawText(text.substring(pos, endPos))
                    if (stack.isEmpty()) doc.prolog.add(node) else stack.last().children.add(node)
                    pos = endPos
                }
                text.startsWith("<?", pos) -> {
                    flushText()
                    val end = text.indexOf("?>", pos + 2)
                    val endPos = if (end < 0) text.length else end + 2
                    val node = RawText(text.substring(pos, endPos) + "\n")
                    if (stack.isEmpty()) doc.prolog.add(node) else stack.last().children.add(node)
                    pos = endPos
                }
                text.startsWith("<!", pos) -> {
                    flushText()
                    var endPos = pos + 2
                    if (text.startsWith("<!DOCTYPE", pos)) {
                        var depth = 0
                        while (endPos < text.length) {
                            when (text[endPos]) {
                                '[' -> depth++
                                ']' -> depth--
                                '>' -> if (depth == 0) { endPos++; break }
                            }
                            endPos++
                        }
                    } else {
                        endPos = (text.indexOf('>', pos + 2).takeIf { it >= 0 } ?: text.length - 1) + 1
                    }
                    val node = RawText(text.substring(pos, endPos) + "\n")
                    if (stack.isEmpty()) doc.prolog.add(node) else stack.last().children.add(node)
                    pos = endPos
                }
                text[pos + 1] == '/' -> {
                    flushText()
                    val end = text.indexOf('>', pos)
                    val name = text.substring(pos + 2, end).trim()
                    val closed = stack.removeLastOrNull()
                    require(closed != null && closed.name == name) {
                        "XML 标签不匹配: 期望 </${closed?.name ?: "?"}> 实际 </$name> (pos=$pos)"
                    }
                    pos = end + 1
                }
                else -> {
                    flushText()
                    val tagEnd = findTagEnd(pos)
                    val raw = text.substring(pos + 1, tagEnd)
                    val selfClosing = raw.endsWith("/")
                    val body = if (selfClosing) raw.dropLast(1) else raw
                    val element = parseTagBody(body)
                    val parent = stack.lastOrNull()
                    if (parent == null) {
                        doc.root = element
                    } else {
                        parent.children.add(element)
                    }
                    if (!selfClosing) stack.addLast(element)
                    pos = tagEnd + 1
                }
            }
        }
        flushText()
        require(stack.isEmpty()) { "XML 存在未闭合标签: ${stack.map { it.name }}" }
        return doc
    }

    private fun findTagEnd(start: Int): Int {
        var i = start + 1
        var inQuote: Char? = null
        while (i < text.length) {
            val c = text[i]
            when {
                inQuote != null -> if (c == inQuote) inQuote = null
                c == '"' || c == '\'' -> inQuote = c
                c == '>' -> return i
            }
            i++
        }
        error("标签未结束 (pos=$start)")
    }

    private fun parseTagBody(body: String): RawElement {
        var i = 0
        fun skipWs() {
            while (i < body.length && body[i].isWhitespace()) i++
        }
        skipWs()
        val nameStart = i
        while (i < body.length && !body[i].isWhitespace()) i++
        val name = body.substring(nameStart, i)
        val attrs = mutableListOf<RawAttr>()
        while (true) {
            skipWs()
            if (i >= body.length) break
            val anStart = i
            while (i < body.length && body[i] != '=' && !body[i].isWhitespace()) i++
            val attrName = body.substring(anStart, i)
            skipWs()
            if (i < body.length && body[i] == '=') {
                i++
                skipWs()
                val quote = body[i]
                require(quote == '"' || quote == '\'') { "属性 $attrName 缺少引号" }
                i++
                val vStart = i
                while (i < body.length && body[i] != quote) i++
                val value = unescape(body.substring(vStart, i))
                i++
                attrs.add(RawAttr(attrName, value))
            } else if (attrName.isNotEmpty()) {
                attrs.add(RawAttr(attrName, ""))
            }
        }
        return RawElement(name, attrs)
    }
}
