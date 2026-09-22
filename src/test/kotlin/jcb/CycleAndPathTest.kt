package jcb

import jcb.calib.CalibStatus
import jcb.calib.CalibrationService
import jcb.calib.CalibrationTransform
import jcb.kin.CycleAnalyzer
import jcb.kin.FrameGraph
import jcb.kin.FrameQueryEngine
import jcb.kin.JointValue
import jcb.urdf.UrdfParser
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test
import java.time.Instant
import kotlin.math.abs

class CycleAndPathTest {
    private fun loopXml(closureX: String) = """
<robot name="loop">
  <link name="base"/><link name="l1"/><link name="l2"/><link name="l3"/>
  <joint name="j1" type="fixed"><origin xyz="1 0 0"/><parent link="base"/><child link="l1"/></joint>
  <joint name="j2" type="fixed"><origin xyz="1 0 0"/><parent link="l1"/><child link="l2"/></joint>
  <joint name="j3" type="fixed"><origin xyz="1 0 0"/><parent link="l2"/><child link="l3"/></joint>
  <joint name="j_loop" type="fixed"><origin xyz="$closureX 0 0"/><parent link="l3"/><child link="base"/></joint>
</robot>"""

    @Test
    fun `一致闭合环残差为零且存在多候选`() {
        // j_loop 从 l3 到 base = -3；闭合 base->l3->base 为 I
        val (_, model) = UrdfParser.parse(loopXml("-3"))
        val g = FrameGraph.build(model).graph
        val report = CycleAnalyzer().analyze(g, emptyMap(), model.joints.associateBy { it.name })
        assertEquals(1, report.cycles.size)
        assertTrue(report.inconsistentCycles.isEmpty())
        assertTrue(report.cycles.single().consistent)
        assertTrue(report.cycles.single().translationResidualM < 1e-9)

        val q = FrameQueryEngine(g).query("base", "l3", emptyMap(), model.joints.associateBy { it.name })
        assertTrue(q.candidates.size >= 2, "应保留至少两条路径，实际 ${q.candidates.size}")
        val spread = q.spread!!
        assertTrue(spread.translationMeters < 1e-9)
        assertTrue(spread.rotationRadians < 1e-9)
    }

    @Test
    fun `不一致环给出非零残差与最小矛盾边集`() {
        val (_, model) = UrdfParser.parse(loopXml("-2.95"))
        val g = FrameGraph.build(model).graph
        val report = CycleAnalyzer().analyze(g, emptyMap(), model.joints.associateBy { it.name })
        assertEquals(1, report.inconsistentCycles.size)
        val cyc = report.inconsistentCycles.single()
        assertTrue(abs(cyc.translationResidualM - 0.05) < 1e-9)
        // 单环：最小矛盾边集 = 环上每条边单独一組（基数 1）
        val sets = report.minContradictingEdgeSets
        assertTrue(sets.all { it.size == 1 })
        assertEquals(cyc.edges.sorted(), sets.flatten().sorted())
    }

    @Test
    fun `固定关节形成的环路不被展开成树丢掉路径`() {
        val (_, model) = UrdfParser.parse(loopXml("-3"))
        val g = FrameGraph.build(model).graph
        val paths = FrameQueryEngine(g).enumeratePaths("base", "l2")
        assertTrue(paths.size >= 2)
        // 每条路径都是简单路径
        assertTrue(paths.all { path ->
            val nodes = mutableListOf(if (path.first().second) path.first().first.parent else path.first().first.child)
            path.forEach { (e, fwd) -> nodes += if (fwd) e.child else e.parent }
            nodes.size == nodes.toSet().size
        })
    }

    private fun calib(id: String, ver: Int, serial: String, x: Double, status: CalibStatus = CalibStatus.APPROVED) =
        CalibrationTransform(
            id = id, version = ver, robotSerial = serial,
            parentFrame = "base", childFrame = "l1",
            xyz = doubleArrayOf(x, 0.0, 0.0),
            rpyRad = DoubleArray(3),
            note = null, validFrom = null, validTo = null,
            createdAt = Instant.parse("2026-01-01T00:00:00Z"),
            status = status, baseVersion = ver - 1,
        )

    @Test
    fun `同一变换URDF与标定重复声明时并列保留`() {
        val (_, model) = UrdfParser.parse(loopXml("-3"))
        val edges = CalibrationService.toEdges(
            CalibrationService.evaluate(
                listOf(calib("c1", 1, "SN-1", 1.0)), "SN-1", Instant.parse("2026-05-01T00:00:00Z")
            )
        )
        val built = FrameGraph.build(model, edges)
        assertTrue(built.graph.issues.any { it.kind == "duplicate_transform" })
        val q = FrameQueryEngine(built.graph).query(
            "base", "l1", emptyMap(), model.joints.associateBy { it.name }
        )
        assertTrue(q.candidates.size >= 2)
        assertTrue(q.candidates.any { it.usesCalibration })
    }

    @Test
    fun `不一致标定制造环矛盾且矛盾边集可定位标定或URDF边`() {
        val (_, model) = UrdfParser.parse(loopXml("-3"))
        val edges = CalibrationService.toEdges(
            CalibrationService.evaluate(
                listOf(calib("c-bad", 1, "SN-1", 1.05)), "SN-1", Instant.now()
            )
        )
        val g = FrameGraph.build(model, edges).graph
        val report = CycleAnalyzer().analyze(g, emptyMap(), model.joints.associateBy { it.name })
        assertTrue(report.inconsistentCycles.isNotEmpty())
        // 至少有一个最小矛盾边集是单条标定边
        assertTrue(report.minContradictingEdgeSets.any { it == listOf("calib:c-bad") })
    }
}
