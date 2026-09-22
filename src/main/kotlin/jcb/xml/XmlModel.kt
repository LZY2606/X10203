package jcb.xml

/**
 * 保真 XML 模型：保留元素、属性、文本、注释与处理指令的原始顺序。
 * 未知元素/属性不会在往返（parse -> write）中丢失。
 */
sealed interface XmlNode {
    val raw: String
}

data class XmlText(override val raw: String) : XmlNode

data class XmlRaw(override val raw: String) : XmlNode

data class XmlElement(
    val tag: String,
    val attributes: LinkedHashMap<String, String> = LinkedHashMap(),
    val children: MutableList<XmlNode> = mutableListOf(),
) : XmlNode {
    override val raw: String get() = buildString { XmlWriter.appendElement(this@XmlElement, this) }

    fun elements(tag: String): List<XmlElement> =
        children.filterIsInstance<XmlElement>().filter { it.tag == tag }

    fun element(tag: String): XmlElement? = elements(tag).firstOrNull()
    fun childElements(): List<XmlElement> = children.filterIsInstance<XmlElement>()

    fun attr(name: String): String? = attributes[name]
    fun attrDouble(name: String): Double? = attributes[name]?.toDoubleOrNull()

    fun setAttr(name: String, value: String) {
        attributes[name] = value
    }

    /** 取（不存在则创建）一个直接子元素，按相邻缩进风格追加。 */
    fun getOrCreateChild(tag: String): XmlElement {
        element(tag)?.let { return it }
        val indent = detectIndent()
        val child = XmlElement(tag)
        children.add(XmlText("\n$indent  "))
        children.add(child)
        children.add(XmlText("\n$indent"))
        return child
    }

    private fun detectIndent(): String {
        val lastText = children.filterIsInstance<XmlText>().lastOrNull()?.raw ?: return ""
        val line = lastText.substringAfterLast('\n')
        return if (line.isBlank()) line else ""
    }
}

data class XmlDocument(
    val prolog: MutableList<XmlNode> = mutableListOf(),
    var root: XmlElement,
    val epilog: MutableList<XmlNode> = mutableListOf(),
) {
    fun toXmlString(): String = XmlWriter.renderDocument(this)
}
