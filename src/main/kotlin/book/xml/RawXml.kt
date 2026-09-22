package book.xml

/**
 * 往返保真的轻量 XML 模型：保留所有元素、属性、注释、声明与文本的原始顺序。
 * 未知元素/属性不会被丢弃，序列化时原样输出。
 */
sealed class RawNode {
    abstract fun render(sb: StringBuilder, indent: Int)
}

data class RawText(val content: String) : RawNode() {
    override fun render(sb: StringBuilder, indent: Int) {
        sb.append(content)
    }
}

data class RawElement(
    var name: String,
    val attributes: MutableList<RawAttr> = mutableListOf(),
    val children: MutableList<RawNode> = mutableListOf(),
) : RawNode() {

    fun attr(name: String): String? = attributes.firstOrNull { it.name == name }?.value

    fun setAttr(name: String, value: String) {
        val existing = attributes.indexOfFirst { it.name == name }
        if (existing >= 0) attributes[existing] = RawAttr(name, value)
        else attributes.add(RawAttr(name, value))
    }

    fun childElements(name: String): List<RawElement> =
        children.mapNotNull { it as? RawElement }.filter { it.name == name }

    fun firstChild(name: String): RawElement? =
        children.mapNotNull { it as? RawElement }.firstOrNull { it.name == name }

    /** 只返回直接子元素（保留顺序）。 */
    fun directElements(): List<RawElement> = children.mapNotNull { it as? RawElement }

    fun deepCopy(): RawElement {
        val copy = RawElement(name, attributes.map { it.copy() }.toMutableList())
        children.forEach { child ->
            when (child) {
                is RawText -> copy.children.add(RawText(child.content))
                is RawElement -> copy.children.add(child.deepCopy())
            }
        }
        return copy
    }

    override fun render(sb: StringBuilder, indent: Int) {
        if (children.isEmpty()) {
            sb.append("  ".repeat(indent)).append('<').append(name)
            attributes.forEach { it.render(sb) }
            sb.append("/>\n")
            return
        }
        val hasElementChildren = children.any { it is RawElement }
        sb.append("  ".repeat(indent)).append('<').append(name)
        attributes.forEach { it.render(sb) }
        if (hasElementChildren) {
            sb.append(">\n")
            children.forEach { it.render(sb, indent + 1) }
            sb.append("  ".repeat(indent)).append("</").append(name).append(">\n")
        } else {
            sb.append('>')
            children.forEach { it.render(sb, indent) }
            sb.append("</").append(name).append(">\n")
        }
    }
}

data class RawAttr(val name: String, val value: String) {
    fun render(sb: StringBuilder) {
        sb.append(' ').append(name).append('=').append('"').append(escapeAttr(value)).append('"')
    }
}

data class RawDocument(
    val prolog: MutableList<RawNode> = mutableListOf(),
    var root: RawElement? = null,
) {
    fun render(): String {
        val sb = StringBuilder()
        prolog.forEach { it.render(sb, 0) }
        root?.render(sb, 0)
        return sb.toString()
    }
}

internal fun escapeAttr(v: String): String =
    v.replace("&", "&amp;").replace("<", "&lt;").replace("\"", "&quot;")

internal fun unescape(v: String): String =
    v.replace("&lt;", "<").replace("&gt;", ">").replace("&quot;", "\"")
        .replace("&apos;", "'").replace("&#10;", "\n").replace("&#13;", "\r")
        .replace("&amp;", "&")
