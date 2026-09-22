package jcb

import jcb.kin.FrameGraph
import jcb.kin.FrameQueryEngine
import jcb.kin.JointUnit
import jcb.kin.JointValue
import jcb.urdf.UrdfParser
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test
import kotlin.math.abs

class MimicTest {
    private val xml = """
<robot name="m">
  <link name="base"/><link name="a"/><link name="b"/><link name="c"/>
  <joint name="ja" type="revolute">
    <origin xyz="0 0 0.1"/><axis xyz="0 0 1"/>
    <limit lower="-3" upper="3" effort="5" velocity="1"/>
    <parent link="base"/><child link="a"/>
  </joint>
  <joint name="jb" type="revolute">
    <origin xyz="0.1 0 0"/><axis xyz="0 0 1"/>
    <mimic joint="ja" multiplier="2" offset="0.1"/>
    <parent link="a"/><child link="b"/>
  </joint>
  <joint name="jc" type="revolute">
    <origin xyz="0.1 0 0"/><axis xyz="0 0 1"/>
    <mimic joint="jb" multiplier="1" offset="0"/>
    <parent link="b"/><child link="c"/>
  </joint>
</robot>"""

    @Test
    fun `mimic链跟随计算`() {
        val (_, model) = UrdfParser.parse(xml)
        val g = FrameGraph.build(model).graph
        val values = mapOf("ja" to JointValue("ja", 0.5, JointUnit.RADIAN))
        val result = FrameQueryEngine(g).query("base", "c", values, model.joints.associateBy { it.name })
        val steps = result.candidates.single().steps
        val byJoint = steps.associateBy { it.jointName }
        assertEquals(0.5, byJoint["ja"]!!.jointValueRadOrM!!, 1e-12)
        assertEquals(2.0 * 0.5 + 0.1, byJoint["jb"]!!.jointValueRadOrM!!, 1e-12)
        assertEquals(2.0 * 0.5 + 0.1, byJoint["jc"]!!.jointValueRadOrM!!, 1e-12)
    }

    @Test
    fun `mimic环被报告且不崩`() {
        val bad = """
<robot name="m">
  <link name="base"/><link name="a"/><link name="b"/>
  <joint name="ja" type="revolute">
    <origin/><axis xyz="0 0 1"/><mimic joint="jb"/>
    <parent link="base"/><child link="a"/>
  </joint>
  <joint name="jb" type="revolute">
    <origin/><axis xyz="0 0 1"/><mimic joint="ja"/>
    <parent link="a"/><child link="b"/>
  </joint>
</robot>"""
        val (_, model) = UrdfParser.parse(bad)
        val issues = FrameGraph.build(model).graph.issues
        assertTrue(issues.any { it.kind == "mimic_cycle" })
        // 不提供关节值：两个关节都是可动的，状态应为缺值
        val q = FrameQueryEngine(FrameGraph.build(model).graph)
            .query("base", "b", emptyMap(), model.joints.associateBy { it.name })
        assertTrue(q.status is jcb.kin.QueryStatus.MissingJointValues)
    }

    @Test
    fun `mimic指向不存在关节`() {
        val bad = """
<robot name="m">
  <link name="base"/><link name="a"/>
  <joint name="ja" type="revolute">
    <origin/><axis xyz="0 0 1"/><mimic joint="ghost"/>
    <parent link="base"/><child link="a"/>
  </joint>
</robot>"""
        val (_, model) = UrdfParser.parse(bad)
        assertTrue(FrameGraph.build(model).graph.issues.any { it.kind == "mimic_chain_unknown" })
    }
}
