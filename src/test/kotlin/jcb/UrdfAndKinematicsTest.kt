package jcb

import jcb.kin.FrameGraph
import jcb.kin.FrameQueryEngine
import jcb.kin.JointUnit
import jcb.kin.JointValue
import jcb.kin.PoseDiff
import jcb.urdf.UrdfParser
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin

class UrdfAndKinematicsTest {
    private val chainXml = """
<robot name="r">
  <link name="base"/><link name="a"/><link name="b"/><link name="tip"/>
  <joint name="j1" type="revolute">
    <origin xyz="1 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3" upper="3" effort="1" velocity="1"/>
    <parent link="base"/><child link="a"/>
  </joint>
  <joint name="j2" type="prismatic">
    <origin xyz="0 0 0"/><axis xyz="1 0 0"/>
    <limit lower="0" upper="1" effort="1" velocity="1"/>
    <parent link="a"/><child link="b"/>
  </joint>
  <joint name="jf" type="fixed">
    <origin xyz="0.5 0 0"/>
    <parent link="b"/><child link="tip"/>
  </joint>
</robot>"""

    @Test
    fun `旋转90度加移动0_2米的FK正确`() {
        val (_, model) = UrdfParser.parse(chainXml)
        val g = FrameGraph.build(model).graph
        val values = mapOf(
            "j1" to JointValue("j1", Math.PI / 2, JointUnit.RADIAN),
            "j2" to JointValue("j2", 0.2, JointUnit.METER),
        )
        val q = FrameQueryEngine(g).query("base", "tip", values, model.joints.associateBy { it.name })
        val t = q.candidates.single().transform
        // 旋转后 x 方向(原 y)累计偏移 0.7；原 x 方向偏移 1
        assertTrue(abs(t[0, 3] - 1.0) < 1e-9, "tx=${t[0, 3]}")
        assertTrue(abs(t[1, 3] - 0.7) < 1e-9, "ty=${t[1, 3]}")
        assertTrue(abs(t[0, 0] - cos(Math.PI / 2)) < 1e-9)
        assertTrue(abs(t[1, 0] - sin(Math.PI / 2)) < 1e-9)
    }

    @Test
    fun `角度输入支持度`() {
        val (_, model) = UrdfParser.parse(chainXml)
        val g = FrameGraph.build(model).graph
        val qDeg = FrameQueryEngine(g).query(
            "base", "a",
            mapOf("j1" to JointValue("j1", 90.0, JointUnit.DEGREE)),
            model.joints.associateBy { it.name }
        )
        assertTrue(abs(qDeg.candidates.single().transform[1, 0] - 1.0) < 1e-9)
    }

    @Test
    fun `缺失关节值有明确状态且不静默按0计算`() {
        val (_, model) = UrdfParser.parse(chainXml)
        val g = FrameGraph.build(model).graph
        val q = FrameQueryEngine(g).query("base", "tip", emptyMap(), model.joints.associateBy { it.name })
        val st = q.status
        assertTrue(st is jcb.kin.QueryStatus.MissingJointValues)
        val missing = (st as jcb.kin.QueryStatus.MissingJointValues).joints
        assertTrue(missing.containsAll(listOf("j1", "j2")))
    }

    @Test
    fun `断链不可达`() {
        val xml = """<robot name="r">
          <link name="a"/><link name="b"/>
          <joint name="jx" type="fixed"><parent link="a"/><child link="ghost"/></joint>
        </robot>"""
        val (_, model) = UrdfParser.parse(xml)
        val built = FrameGraph.build(model)
        assertTrue(built.graph.issues.any { it.kind == "broken_link" })
        val q = FrameQueryEngine(built.graph).query("a", "b", emptyMap(), emptyMap())
        assertTrue(q.status is jcb.kin.QueryStatus.Unreachable)
    }

    @Test
    fun `重复frame被报告`() {
        val xml = """<robot name="r">
          <link name="dup"/><link name="dup"/>
          <joint name="j" type="fixed"><parent link="dup"/><child link="dup"/></joint>
        </robot>"""
        val (_, model) = UrdfParser.parse(xml)
        val issues = FrameGraph.build(model).graph.issues
        assertTrue(issues.any { it.kind == "duplicate_frame" && it.message.contains("link") })
    }
}
