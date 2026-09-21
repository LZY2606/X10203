package coordbook

import kotlin.math.PI
import kotlin.math.abs
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue
import kotlin.test.assertNotNull
import kotlin.test.assertNull

class KinematicsTest {

    private fun buildGraph(xml: String): Triple<UrdfDocument, FrameGraph, List<DocumentIssue>> {
        val doc = UrdfParser.parse(xml)
        val (g, issues) = GraphBuilder.build(doc)
        return Triple(doc, g, issues)
    }

    private val mimicChainXml = """<?xml version="1.0"?>
<robot name="m">
  <link name="b"/><link name="s"/><link name="e"/><link name="w"/>
  <joint name="j1" type="revolute">
    <parent link="b"/><child link="s"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3" upper="3" effort="1" velocity="1"/>
  </joint>
  <joint name="j2" type="revolute">
    <parent link="s"/><child link="e"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-3" upper="3" effort="1" velocity="1"/>
  </joint>
  <joint name="j3" type="revolute">
    <parent link="e"/><child link="w"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-3" upper="3" effort="1" velocity="1"/>
    <mimic joint="j2" multiplier="-1" offset="0.2"/>
  </joint>
</robot>
"""

    @Test
    fun testMimicChainDerivesValuesAndKeepsUnits() {
        val (doc, graph, _) = buildGraph(mimicChainXml)
        val result = Kinematics.query(graph, doc, "b", "w", mapOf("j1" to 0.0, "j2" to 1.0))
        assertTrue(result.ok, result.error)
        val report = result.report!!
        assertEquals(1, report.candidates.size)
        val steps = report.candidates.single().steps
        val j3 = steps.single { it.jointName == "j3" }
        assertEquals(-0.8, j3.jointValue!!, 1e-12)
        assertEquals("rad", j3.valueUnit)
        assertEquals("mimic:j2", j3.valueOrigin)
        assertEquals("snapshot", steps.first { it.jointName == "j1" }.valueOrigin)
        // j3 绕 y 旋转 -0.8
        val t = Transform(j3.transform.flatten().toDoubleArray())
        // 平移为 0
        assertEquals(0.0, t.m[3], 1e-12)
        // R_y(-0.8) 的 [0][0] = cos(-0.8)
        assertEquals(kotlin.math.cos(-0.8), t.m[0], 1e-12)
    }

    @Test
    fun testMimicCycleIsReportedNotExpanded() {
        val xml = """<?xml version="1.0"?>
<robot name="cyc">
  <link name="a"/><link name="b"/><link name="c"/>
  <joint name="j1" type="revolute">
    <parent link="a"/><child link="b"/><axis xyz="0 0 1"/>
    <limit lower="0" upper="1" effort="1" velocity="1"/>
    <mimic joint="j2"/>
  </joint>
  <joint name="j2" type="revolute">
    <parent link="b"/><child link="c"/><axis xyz="0 0 1"/>
    <limit lower="0" upper="1" effort="1" velocity="1"/>
    <mimic joint="j1"/>
  </joint>
</robot>
"""
        val doc = UrdfParser.parse(xml)
        assertTrue(doc.issues.any { it.code == "MIMIC_CYCLE" })
    }

    @Test
    fun testMissingJointValueHasExplicitStatus() {
        val (doc, graph, _) = buildGraph(mimicChainXml)
        val result = Kinematics.query(graph, doc, "b", "w", mapOf("j1" to 0.1))
        val report = result.report!!
        assertNotNull(report.missingJoint)
        // j2 缺失，j3 沿 mimic 链同样缺值
        assertTrue(report.missingJoint.jointName in listOf("j2", "j3"))
        assertTrue(report.candidates.isEmpty())
    }

    @Test
    fun testAngleUnitsRadiansInMatrix() {
        // 单关节绕 z 旋转 90 度 = pi/2；验证矩阵列向量
        val xml = """<?xml version="1.0"?>
<robot name="u">
  <link name="a"/><link name="b"/>
  <joint name="j" type="revolute">
    <parent link="a"/><child link="b"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3" upper="3" effort="1" velocity="1"/>
  </joint>
</robot>
"""
        val (doc, graph, _) = buildGraph(xml)
        val result = Kinematics.query(graph, doc, "a", "b", mapOf("j" to PI / 2))
        val t = result.report!!.consensus!!.flatten()
        // R_z(pi/2) 标准旋转矩阵
        assertEquals(0.0, t[0], 1e-12)
        assertEquals(-1.0, t[1], 1e-12)
        assertEquals(1.0, t[4], 1e-12)
        assertEquals(0.0, t[5], 1e-12)
    }

    private val consistentLoopXml = """<?xml version="1.0"?>
<robot name="loop">
  <link name="a"/><link name="b"/><link name="c"/>
  <joint name="ab" type="fixed"><parent link="a"/><child link="b"/>
    <origin xyz="1 0 0" rpy="0 0 0"/></joint>
  <joint name="bc" type="fixed"><parent link="b"/><child link="c"/>
    <origin xyz="0 1 0" rpy="0 0 0"/></joint>
  <joint name="ac" type="fixed"><parent link="a"/><child link="c"/>
    <origin xyz="1 1 0" rpy="0 0 0"/></joint>
</robot>
"""

    @Test
    fun testConsistentLoopAndMultipleAgreeingPaths() {
        val (doc, graph, _) = buildGraph(consistentLoopXml)
        val report = Cycles.analyze(graph)
        assertTrue(report.hasCycle)
        assertTrue(report.residuals.all { it.consistent })
        assertEquals(emptyList(), report.minimalContradictingEdges)
        // a->c 有两条一致路径，均保留
        val q = Kinematics.query(graph, doc, "a", "c", emptyMap())
        val r = q.report!!
        assertEquals(2, r.candidates.size)
        assertTrue(r.consistent)
        assertEquals(0.0, r.maxTranslationErrorM, 1e-12)
        // 共识矩阵平移 (1,1,0)
        val m = r.consensus!!.flatten()
        assertEquals(1.0, m[3], 1e-12)
        assertEquals(1.0, m[7], 1e-12)
    }

    private val inconsistentLoopXml = """<?xml version="1.0"?>
<robot name="badloop">
  <link name="a"/><link name="b"/><link name="c"/>
  <joint name="ab" type="fixed"><parent link="a"/><child link="b"/>
    <origin xyz="1 0 0" rpy="0 0 0"/></joint>
  <joint name="bc" type="fixed"><parent link="b"/><child link="c"/>
    <origin xyz="0 1 0" rpy="0 0 0"/></joint>
  <joint name="ca" type="fixed"><parent link="c"/><child link="a"/>
    <origin xyz="0 -1.02 0" rpy="0 0 0"/></joint>
</robot>
"""

    @Test
    fun testInconsistentLoopResidualAndMinimalContradictingSet() {
        val (_, graph, _) = buildGraph(inconsistentLoopXml)
        val report = Cycles.analyze(graph)
        assertTrue(report.inconsistent)
        assertTrue(report.residuals.single().translationErrorM > 0.01)
        // 删除一条边即可恢复一致；最小矛盾边集基数为 1，且确定性地指向 ca
        assertEquals(1, report.minimalContradictingEdges.size)
        assertEquals("urdf:joint::ca", report.minimalContradictingEdges.single())
    }

    @Test
    fun testMultipleInconsistentEdgesNeedSetOfTwo() {
        val xml = """<?xml version="1.0"?>
<robot name="twoloop">
  <link name="a"/><link name="b"/><link name="c"/><link name="d"/>
  <joint name="ab" type="fixed"><parent link="a"/><child link="b"/>
    <origin xyz="1 0 0" rpy="0 0 0"/></joint>
  <joint name="bc" type="fixed"><parent link="b"/><child link="c"/>
    <origin xyz="0 1 0" rpy="0 0 0"/></joint>
  <joint name="ca" type="fixed"><parent link="c"/><child link="a"/>
    <origin xyz="0 -1.05 0" rpy="0 0 0"/></joint>
  <joint name="cd" type="fixed"><parent link="c"/><child link="d"/>
    <origin xyz="1 0 0" rpy="0 0 0"/></joint>
  <joint name="da" type="fixed"><parent link="d"/><child link="a"/>
    <origin xyz="-1.05 -1 0" rpy="0 0 0"/></joint>
</robot>
"""
        val (_, graph, _) = buildGraph(xml)
        val report = Cycles.analyze(graph)
        assertTrue(report.inconsistent)
        assertEquals(2, report.minimalContradictingEdges.size)
    }

    @Test
    fun testParallelPathsKeepAllCandidatesWithErrors() {
        val xml = """<?xml version="1.0"?>
<robot name="dup">
  <link name="a"/><link name="b"/>
  <joint name="j1" type="fixed"><parent link="a"/><child link="b"/>
    <origin xyz="1 0 0" rpy="0 0 0"/></joint>
  <joint name="j2" type="fixed"><parent link="a"/><child link="b"/>
    <origin xyz="1 0.03 0" rpy="0 0 0"/></joint>
</robot>
"""
        val (doc, graph, issues) = buildGraph(xml)
        assertTrue(issues.any { it.code == "DUPLICATE_TRANSFORM" })
        val r = Kinematics.query(graph, doc, "a", "b", emptyMap()).report!!
        assertEquals(2, r.candidates.size)
        assertTrue(!r.consistent)
        assertEquals(0.03, r.maxTranslationErrorM, 1e-9)
        assertNull(r.consensus)
    }

    @Test
    fun testDuplicateFrameNameIsAmbiguousAndBrokenChainReported() {
        val xml = """<?xml version="1.0"?>
<robot name="dup2">
  <link name="x"/><link name="x"/>
  <link name="y"/>
  <joint name="jx" type="fixed"><parent link="x"/><child link="y"/>
    <origin xyz="0 0 0" rpy="0 0 0"/></joint>
  <joint name="jz" type="fixed"><parent link="y"/><child link="ghost"/>
    <origin xyz="0 0 0" rpy="0 0 0"/></joint>
</robot>
"""
        val doc = UrdfParser.parse(xml)
        assertTrue(doc.issues.any { it.code == "DUPLICATE_FRAME" })
        assertTrue(doc.issues.any { it.code == "BROKEN_CHAIN" })
        val (_, _, graphIssues) = Triple(doc, GraphBuilder.build(doc).first, GraphBuilder.build(doc).second)
        assertTrue(graphIssues.any { it.code == "DUPLICATE_FRAME" })
    }
}
