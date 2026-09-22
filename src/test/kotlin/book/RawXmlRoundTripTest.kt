package book

import book.calib.CalibrationEdge
import book.calib.CalibrationVersion
import book.calib.UrdfExporter
import book.urdf.UrdfParser
import book.xml.RawXmlParser
import kotlin.test.Test
import kotlin.test.assertContains
import kotlin.test.assertEquals
import kotlin.test.assertTrue

class RawXmlRoundTripTest {

    private val xml = """
<?xml version="1.0"?>
<!-- 顶部注释 -->
<robot name="r" custom:foo="bar">
  <manufacturer:note vendor="acme">未知元素内容</manufacturer:note>
  <link name="a"><visual><geometry><mesh filename="a.stl"/></geometry></visual></link>
  <joint name="j" type="fixed" custom:weird="1">
    <parent link="a"/><child link="b"/>
    <origin xyz="1 2 3" rpy="0 0 0"/>
  </joint>
  <link name="b"/>
</robot>
""".trimIndent()

    @Test
    fun `unknown elements attributes comments and order survive round trip`() {
        val rendered = UrdfParser.parse(xml).document.render()
        assertContains(rendered, "manufacturer:note")
        assertContains(rendered, """vendor="acme"""")
        assertContains(rendered, "顶部注释")
        assertContains(rendered, """custom:foo="bar"""")
        assertContains(rendered, """custom:weird="1"""")
        assertContains(rendered, "<mesh")
        // 顺序：未知元素仍在 link 之前
        assertTrue(rendered.indexOf("manufacturer:note") < rendered.indexOf("""name="a""""))
    }

    @Test
    fun `approved override export keeps unknown content and marks derived file`() {
        val model = UrdfParser.parse(xml)
        val calib = CalibrationVersion(
            id = "c1", robotSerial = "SN", version = 1, note = "n",
            createdAt = "2026-01-01T00:00:00Z",
            validFrom = "2026-01-01T00:00:00Z", validUntil = "2030-01-01T00:00:00Z",
            draft = false, approved = true,
            edges = listOf(CalibrationEdge(
                id = "e1", kind = "override", jointName = "j",
                parentFrame = "a", childFrame = "b",
                xyz = listOf(0.5, 0.0, 0.0), rpyRad = listOf(0.0, 0.0, 0.0))),
        )
        val result = UrdfExporter.applyApproved(model, calib, setOf("e1"))
        assertContains(result.xml, """xyz="0.5 0 0"""")
        assertContains(result.xml, "manufacturer:note")
        assertContains(result.xml, "顶部注释")
        assertContains(result.xml, "并非原始 URDF")
        assertEquals(listOf("e1"), result.appliedEdges)
    }

    @Test
    fun `unapproved edges are not applied`() {
        val model = UrdfParser.parse(xml)
        val calib = CalibrationVersion(
            id = "c1", robotSerial = "SN", version = 1, note = "",
            createdAt = "2026-01-01T00:00:00Z",
            validFrom = "2026-01-01T00:00:00Z", validUntil = "2030-01-01T00:00:00Z",
            draft = false, approved = true,
            edges = listOf(CalibrationEdge(
                id = "e1", kind = "new", jointName = null,
                parentFrame = "a", childFrame = "c",
                xyz = listOf(1.0, 0.0, 0.0), rpyRad = listOf(0.0, 0.0, 0.0))),
        )
        val result = UrdfExporter.applyApproved(model, calib, emptySet())
        assertEquals(emptyList(), result.appliedEdges)
        assertEquals(listOf("e1"), result.skippedUnapproved)
        assertTrue("calib_e1" !in result.xml)
    }
}
