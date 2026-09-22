package jcb

import jcb.xml.parseXml
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

class XmlRoundTripTest {
    private val xml = """<?xml version="1.0" encoding="UTF-8"?>
<!-- 前导注释 -->
<robot name="r" xmlns:jcb="http://x" jcb:serial="SN-1">
  <jcb:custom type="field">未知元素文本</jcb:custom>
  <link name="base" jcb:marker="地脚">
    <visual>
      <geometry><box size="1 2 3"/></geometry>
      <jcb:extra a="1" b="2"/>
    </visual>
  </link>
  <joint name="j" type="revolute" custom-attr="保留">
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <parent link="base"/><child link="tip"/>
  </joint>
</robot>
"""

    @Test
    fun `未知元素属性注释顺序往返保真`() {
        val doc = parseXml(xml)
        val out = doc.toXmlString()
        assertEquals(xml, out)
        val root = doc.root
        assertEquals("SN-1", root.attr("jcb:serial"))
        val custom = root.childElements().first { it.tag == "jcb:custom" }
        assertEquals("field", custom.attr("type"))
        val link = root.element("link")!!
        assertEquals("地脚", link.attr("jcb:marker"))
        assertTrue(link.element("visual")!!.element("jcb:extra") != null)
        assertTrue(doc.prolog.any { it.raw.contains("前导注释") })
    }

    @Test
    fun `实体与CDATA解析`() {
        val doc = parseXml("""<root a="1 &lt; 2 &amp; 3"><![CDATA[x <y> &z]]></root>""")
        assertEquals("1 < 2 & 3", doc.root.attr("a"))
        assertEquals("<![CDATA[x <y> &z]]>", doc.root.children.first().raw)
    }
}
