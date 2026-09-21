package coordbook

import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertContains
import kotlin.test.assertTrue

class XmlRoundTripTest {

    private val xml = """<?xml version="1.0" encoding="UTF-8"?>
<robot name="r">
  <!-- 厂商注释：往返必须保留 -->
  <link name="a">
    <vendor:custom key="v&amp;1" data="x='1'">
      <nested unknown-attr="9">文本 &lt;保留&gt;</nested>
      <![CDATA[raw <cdata> & stuff]]>
    </vendor:custom>
  </link>
  <joint name="ja" type="fixed">
    <parent link="a"/><child link="a"/>
    <origin xyz="1 2 3" rpy="0.1 0.2 0.3" custom-attr="keep"/>
  </joint>
</robot>
"""

    @Test
    fun testUnknownContentRoundTripsExactly() {
        val (root, declaration) = XmlParser.parse(xml)
        val out = root.toXmlString(declaration)
        assertEquals(xml, out)
    }

    @Test
    fun testUnknownElementsReportedAndParsedFieldsAvailable() {
        val doc = UrdfParser.parse(xml)
        assertEquals("r", doc.root.attr("name"))
        assertEquals(1, doc.links.size)
        assertEquals(1, doc.joints.size)
        // 未知元素仍在 DOM 中
        val link = doc.links.getValue("link::a")
        val custom = link.element.child("vendor:custom")
        assertEquals("v&1", custom?.attr("key"))
    }

    @Test
    fun testPatchOnlyTouchesKnownOriginAndPreservesRest() {
        val doc = UrdfParser.parse(xml)
        val patch = CalibPatch(
            id = 1, versionId = 1, kind = PatchKind.JOINT_ORIGIN_RPY,
            targetJoint = "ja", parentFrame = null, childFrame = null,
            xyz = null, rpyRadians = doubleArrayOf(0.0, 0.0, 0.5),
            status = PatchStatus.APPROVED, note = null, createdAt = "",
        )
        val (exported, applied) = XmlPatcher.applyApprovedPatches(doc, listOf(patch))
        assertEquals(1, applied.size)
        assertContains(exported, """rpy="0 0 0.5"""")
        assertContains(exported, "custom-attr=\"keep\"")
        assertContains(exported, "vendor:custom")
        assertContains(exported, "raw <cdata> & stuff")
        assertContains(exported, "厂商注释：往返必须保留")
        // xyz 未被改动
        assertContains(exported, """xyz="1 2 3"""")
    }

    @Test
    fun testPendingPatchesAreNotExported() {
        val doc = UrdfParser.parse(xml)
        val patch = CalibPatch(
            id = 1, versionId = 1, kind = PatchKind.JOINT_ORIGIN_RPY,
            targetJoint = "ja", parentFrame = null, childFrame = null,
            xyz = null, rpyRadians = doubleArrayOf(0.0, 0.0, 0.5),
            status = PatchStatus.PENDING, note = null, createdAt = "",
        )
        val (exported, applied) = XmlPatcher.applyApprovedPatches(doc, listOf(patch))
        assertEquals(0, applied.size)
        assertContains(exported, """rpy="0.1 0.2 0.3"""")
    }
}
