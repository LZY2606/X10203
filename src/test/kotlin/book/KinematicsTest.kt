package book

import book.calib.CalibrationEdge
import book.calib.CalibrationVersion
import book.kin.GraphAnalyzer
import book.kin.GraphBuilder
import book.urdf.UrdfParser
import kotlin.math.abs
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue

class KinematicsTest {

    private fun links(names: List<String>) = names.joinToString("\n") { """<link name="$it"/>""" }
    private fun fixed(name: String, p: String, c: String, xyz: String = "0 0 0") =
        """<joint name="$name" type="fixed"><parent link="$p"/><child link="$c"/>
          |<origin xyz="$xyz" rpy="0 0 0"/></joint>""".trimMargin()
    private fun revolute(name: String, p: String, c: String, xyz: String = "0 0 0",
                         axis: String = "0 0 1", mimic: String? = null) =
        """<joint name="$name" type="revolute"><parent link="$p"/><child link="$c"/>
          |<origin xyz="$xyz" rpy="0 0 0"/><axis xyz="$axis"/>
          |${if (mimic != null) """<mimic joint="$mimic"/>""" else ""}
          |<limit lower="-3.14" upper="3.14" effort="10" velocity="1"/></joint>""".trimMargin()

    private fun wrapRobot(body: String) =
        """<?xml version="1.0"?><robot name="t">$body</robot>"""

    @Test
    fun `mimic chain derives joint values`() {
        val xml = wrapRobot(
            links(listOf("a", "b")) +
            revolute("j1", "a", "b") +
            revolute("j2", "a", "b", mimic = "j1")
        )
        val g = GraphBuilder.build(UrdfParser.parse(xml), null)
        val values = g.resolveJointValues(mapOf("j1" to 0.5), emptyMap())
        assertEquals(0.5, values["j2"]!!.value!!, 1e-9)
        assertTrue(values["j2"]!!.fromMimic)
    }

    @Test
    fun `mimic cycle is reported and values missing`() {
        val xml = wrapRobot(
            links(listOf("a", "b")) +
            revolute("j1", "a", "b", mimic = "j2") +
            revolute("j2", "a", "b", mimic = "j1")
        )
        val model = UrdfParser.parse(xml)
        assertTrue(model.issues.any { it.code == "MIMIC_CYCLE" })
        val g = GraphBuilder.build(model, null)
        val values = g.resolveJointValues(mapOf("j1" to 0.1, "j2" to 0.2), emptyMap())
        assertEquals("MIMIC_CYCLE", values["j2"]!!.status)
    }

    @Test
    fun `consistent fixed loop has zero residual`() {
        // a->b (1,0,0); a->c (0,1,0); c->b (1,-1,0): 环 a-b-c-a 闭合
        val xml = wrapRobot(
            links(listOf("a", "b", "c")) +
            fixed("j1", "a", "b", "1 0 0") +
            fixed("j2", "a", "c", "0 1 0") +
            fixed("j3", "c", "b", "1 -1 0")
        )
        val g = GraphBuilder.build(UrdfParser.parse(xml), null, 1e-9, 1e-9)
        val report = GraphAnalyzer(g).report()
        assertTrue(report.loops.isNotEmpty(), "应检测到环")
        assertTrue(report.loops.all { it.consistent },
            "所有环应一致，实际: ${report.loops.map { it.translationError }}")
    }

    @Test
    fun `inconsistent loop reports residual and minimum contradiction edge`() {
        // a->b 应为 (1,0,0)，但另一路径给出 c->b=(1.1,-1,0) => 0.1m 残差
        val xml = wrapRobot(
            links(listOf("a", "b", "c")) +
            fixed("j1", "a", "b", "1 0 0") +
            fixed("j2", "a", "c", "0 1 0") +
            fixed("j3", "c", "b", "1.1 -1 0")
        )
        val g = GraphBuilder.build(UrdfParser.parse(xml), null, 1e-9, 1e-9)
        val report = GraphAnalyzer(g).report()
        val bad = report.loops.filterNot { it.consistent }
        assertTrue(bad.isNotEmpty(), "应检测到矛盾环")
        assertTrue(bad.all { abs(it.translationError - 0.1) < 1e-9 },
            "残差应为 0.1m，实际 ${bad.map { it.translationError }}")
        val mc = report.minimumContradiction!!
        assertEquals(1, mc.size)
        assertTrue(mc.edgeSet.single() in setOf("urdf:j1", "urdf:j2", "urdf:j3"))
    }

    @Test
    fun `multiple consistent paths keep all candidates with spread`() {
        // a->b 有两条几何一致的路径：直达 j1=1m；绕行 c: j2+j3 合计 1m
        val xml = wrapRobot(
            links(listOf("a", "b", "c")) +
            fixed("j1", "a", "b", "1 0 0") +
            fixed("j2", "a", "c", "0.3 0 0") +
            fixed("j3", "c", "b", "0.7 0 0")
        )
        val g = GraphBuilder.build(UrdfParser.parse(xml), null)
        val result = g.query("a", "b", emptyMap())
        assertEquals("MULTIPLE", result.status)
        assertEquals(2, result.candidates.size)
        assertTrue(result.candidates.all { it.complete })
        assertEquals(0.0, result.spread!!.translationMeters, 1e-9)
    }

    @Test
    fun `degrees input converts to radians internally`() {
        val xml = wrapRobot(
            links(listOf("a", "b")) + revolute("j1", "a", "b")
        )
        val g = GraphBuilder.build(UrdfParser.parse(xml), null)
        val values = g.resolveJointValues(mapOf("j1" to 90.0), mapOf("j1" to "deg"))
        val rad = values["j1"]!!.value!!
        assertEquals(Math.PI / 2, rad, 1e-9)
        assertEquals("rad", values["j1"]!!.unit)

        val radOnly = g.resolveJointValues(mapOf("j1" to 1.0), mapOf("j1" to "rad"))
        assertEquals(1.0, radOnly["j1"]!!.value!!, 1e-12)
    }

    @Test
    fun `missing joint value yields incomplete candidate with explicit status`() {
        val xml = wrapRobot(
            links(listOf("a", "b")) + revolute("j1", "a", "b", "0 0 1")
        )
        val g = GraphBuilder.build(UrdfParser.parse(xml), null)
        val values = g.resolveJointValues(emptyMap(), emptyMap())
        assertEquals("MISSING", values["j1"]!!.status)
        val result = g.query("a", "b", emptyMap())
        assertTrue(result.candidates.none { it.complete })
        assertTrue(result.candidates.first().problems.any { "MISSING" in it })
    }

    @Test
    fun `dangling endpoint is reported as broken link`() {
        val xml = wrapRobot(
            links(listOf("a")) + fixed("j1", "ghost", "a")
        )
        val g = GraphBuilder.build(UrdfParser.parse(xml), null)
        val report = GraphAnalyzer(g).report()
        assertTrue(report.brokenLinks.any { "ghost" in it })
        val result = g.query("a", "ghost", emptyMap())
        assertEquals("UNIQUE", result.status) // 外部节点仍可追踪，但状态面板报告断链
    }

    @Test
    fun `duplicate frame names produce distinct occurrences`() {
        val xml = wrapRobot("""
            <link name="x"/><link name="x"/>
            ${fixed("j1", "x", "x", "1 0 0")}
        """.trimIndent())
        val model = UrdfParser.parse(xml)
        assertTrue(model.issues.any { it.code == "DUPLICATE_LINK" })
        val g = GraphBuilder.build(model, null)
        assertTrue(g.nodesByName("x").size >= 2)
    }

    @Test
    fun `new calibration edge duplicating a urdf edge is flagged`() {
        val xml = wrapRobot(
            links(listOf("a", "b")) + fixed("j1", "a", "b", "1 0 0")
        )
        val calib = CalibrationVersion(
            id = "c", robotSerial = "S", version = 1, note = "",
            createdAt = "2026-01-01T00:00:00Z",
            validFrom = "2026-01-01T00:00:00Z", validUntil = "2030-01-01T00:00:00Z",
            draft = false, approved = true,
            edges = listOf(CalibrationEdge(
                id = "dup", kind = "new", jointName = null,
                parentFrame = "a", childFrame = "b",
                xyz = listOf(1.0, 0.0, 0.0), rpyRad = listOf(0.0, 0.0, 0.0))),
        )
        val g = GraphBuilder.build(UrdfParser.parse(xml), calib)
        val report = GraphAnalyzer(g).report()
        assertTrue(report.duplicateDeclarations.any { "dup" in it })
    }
}
