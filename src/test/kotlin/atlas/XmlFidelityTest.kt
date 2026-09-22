package atlas

import atlas.urdf.PatchApplier
import atlas.urdf.UrdfParser
import atlas.store.PatchOp
import atlas.xml.XmlParser
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue

class XmlFidelityTest {
    private val xml = """<?xml version="1.0"?>
<robot name="r" xmlns:acme="http://acme.example" customAttr="keep-me">
  <acme:cellMeta floor="B2"/>
  <!-- a comment -->
  <link name="a"><inertial><mass value="1.5"/></inertial></link>
  <link name="b"/>
  <joint name="j1" type="fixed">
    <parent link="a"/><child link="b"/>
    <origin xyz="0 0 1" rpy="0 0 0"/>
    <acme:vendorTag serial="X9">text-body</acme:vendorTag>
  </joint>
  <gazebo reference="a"><material>Gazebo/Blue</material></gazebo>
</robot>
"""

    @Test
    fun `unknown elements and attributes survive import`() {
        val robot = UrdfParser.parse(xml)
        assertEquals("r", robot.name)
        assertEquals(2, robot.links.size)
        assertEquals(1, robot.joints.size)
        val root = robot.document.root
        assertEquals("keep-me", root.attr("customAttr"))
        assertTrue(root.childElements("acme:cellMeta").isNotEmpty())
        assertTrue(root.childElements("gazebo").isNotEmpty())
        val j1 = root.childElements("joint").single()
        assertTrue(j1.childElements("acme:vendorTag").isNotEmpty())
    }

    @Test
    fun `original child order is preserved`() {
        val robot = UrdfParser.parse(xml)
        val names = robot.document.root.children.filterIsInstance<atlas.xml.XmlElement>().map { it.name }
        assertEquals(listOf("acme:cellMeta", "link", "link", "joint", "gazebo"), names)
    }

    @Test
    fun `round trip serialize parse is stable`() {
        val doc1 = XmlParser.parse(xml)
        val s1 = doc1.serialize()
        val doc2 = XmlParser.parse(s1)
        val s2 = doc2.serialize()
        assertEquals(s1, s2)
        // key unknown content still present in serialized form
        assertTrue(s1.contains("acme:cellMeta"))
        assertTrue(s1.contains("customAttr=\"keep-me\""))
        assertTrue(s1.contains("<!-- a comment -->"))
        assertTrue(s1.contains("Gazebo/Blue"))
    }

    @Test
    fun `approved patch export keeps unknown content`() {
        val robot = UrdfParser.parse(xml)
        val patched = PatchApplier.apply(
            robot.document,
            listOf(PatchOp(type = "setOrigin", joint = "j1", xyz = listOf(0.0, 0.0, 2.0), rpy = listOf(0.0, 0.0, 0.0)))
        )
        val out = patched.serialize()
        assertTrue(out.contains("xyz=\"0.0 0.0 2.0\""), out)
        assertTrue(out.contains("acme:cellMeta"))
        assertTrue(out.contains("acme:vendorTag"))
        assertTrue(out.contains("customAttr=\"keep-me\""))
        // re-parse to confirm validity
        val reparsed = UrdfParser.parse(out)
        assertEquals(2.0, reparsed.joints.single().origin.xyz.z)
    }
}
