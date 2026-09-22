package coordbook

import kotlin.math.PI
import kotlin.math.abs
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFailsWith
import kotlin.test.assertTrue

private fun robot(xml: String) = UrdfParser.parse(xml)

private const val CHAIN = """
<robot name="r">
  <link name="a"/><link name="b"/><link name="c"/>
  <joint name="j1" type="revolute"><parent link="a"/><child link="b"/>
    <origin xyz="1 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/></joint>
  <joint name="j2" type="revolute"><parent link="b"/><child link="c"/>
    <origin xyz="1 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/></joint>
</robot>"""

class MimicTest {
    @Test fun mimicChainResolves() {
        val r = robot("""
        <robot name="m">
          <link name="a"/><link name="b"/><link name="c"/><link name="d"/>
          <joint name="j1" type="revolute"><parent link="a"/><child link="b"/><axis xyz="0 0 1"/></joint>
          <joint name="j2" type="revolute"><parent link="b"/><child link="c"/><axis xyz="0 0 1"/>
            <mimic joint="j1" multiplier="2" offset="0.5"/></joint>
          <joint name="j3" type="revolute"><parent link="c"/><child link="d"/><axis xyz="0 0 1"/>
            <mimic joint="j2" multiplier="1" offset="0.25"/></joint>
        </robot>""")
        val v = Kinematics.resolveJointValues(r, mapOf("j1" to 0.3))
        assertEquals(0.3, v["j1"]!!.first!!, 1e-12)
        assertEquals(2 * 0.3 + 0.5, v["j2"]!!.first!!, 1e-12)
        assertEquals(2 * 0.3 + 0.5 + 0.25, v["j3"]!!.first!!, 1e-12)
    }

    @Test fun mimicCycleDetected() {
        val r = robot("""
        <robot name="mc">
          <link name="a"/><link name="b"/><link name="c"/>
          <joint name="j1" type="revolute"><parent link="a"/><child link="b"/>
            <mimic joint="j2" multiplier="1" offset="0"/></joint>
          <joint name="j2" type="revolute"><parent link="b"/><child link="c"/>
            <mimic joint="j1" multiplier="1" offset="0"/></joint>
        </robot>""")
        val v = Kinematics.resolveJointValues(r, emptyMap())
        assertEquals("mimic_cycle", v["j1"]!!.second)
        assertEquals("mimic_cycle", v["j2"]!!.second)
    }
}

class CycleTest {
    private fun ctx(robot: UrdfRobot, cals: List<Calibration>, jv: Map<String, Double> = emptyMap()) =
        GraphContext(robot, "S1", jv, cals, null)

    @Test fun consistentCycleResidualZero() {
        val r = robot(CHAIN)
        // close c -> a with exact inverse of chain at q=0: translation (-2,0,0)
        val cal = Calibration(1, "S1", "loop", "c", "a",
            doubleArrayOf(-2.0, 0.0, 0.0), doubleArrayOf(0.0, 0.0, 0.0), "rad", null, null, 1)
        val edges = Kinematics.buildEdges(ctx(r, listOf(cal), mapOf("j1" to 0.0, "j2" to 0.0)))
        val cycles = Kinematics.cycleResiduals(edges)
        assertEquals(1, cycles.size)
        assertTrue(cycles[0].consistent, "residual=${cycles[0].translationError}")
        assertTrue(Kinematics.minimalContradictionEdgeSet(cycles).isEmpty())
    }

    @Test fun inconsistentCycleReportsMinimalContradictionSet() {
        val r = robot(CHAIN)
        val cal = Calibration(1, "S1", "loop", "c", "a",
            doubleArrayOf(-2.0, 0.0, 0.01), doubleArrayOf(0.0, 0.0, 0.0), "rad", null, null, 1)
        val edges = Kinematics.buildEdges(ctx(r, listOf(cal), mapOf("j1" to 0.0, "j2" to 0.0)))
        val cycles = Kinematics.cycleResiduals(edges)
        assertEquals(1, cycles.size)
        assertTrue(!cycles[0].consistent)
        assertEquals(0.01, cycles[0].translationError, 1e-9)
        // single inconsistent cycle: any one edge is a min hitting set; the calibration edge is the culprit here
        val mces = Kinematics.minimalContradictionEdgeSet(cycles)
        assertEquals(1, mces.size)
    }

    @Test fun duplicateDeclarationFormsTwoCycle() {
        val r = robot("""
        <robot name="d"><link name="a"/><link name="b"/>
          <joint name="j1" type="fixed"><parent link="a"/><child link="b"/><origin xyz="0 0 1" rpy="0 0 0"/></joint>
        </robot>""")
        val cal = Calibration(1, "S1", "dup", "a", "b",
            doubleArrayOf(0.0, 0.0, 1.0), doubleArrayOf(0.0, 0.0, 0.0), "rad", null, null, 1)
        val edges = Kinematics.buildEdges(ctx(r, listOf(cal)))
        assertTrue(edges.all { it.duplicate })
        val cycles = Kinematics.cycleResiduals(edges)
        assertEquals(1, cycles.size)
        assertTrue(cycles[0].consistent)
    }
}

class MultiPathTest {
    @Test fun parallelConsistentPathsAllKept() {
        // diamond: a->b->d and a->c->d, both consistent
        val r = robot("""
        <robot name="dia">
          <link name="a"/><link name="b"/><link name="c"/><link name="d"/>
          <joint name="j1" type="fixed"><parent link="a"/><child link="b"/><origin xyz="1 0 0" rpy="0 0 0"/></joint>
          <joint name="j2" type="fixed"><parent link="b"/><child link="d"/><origin xyz="0 1 0" rpy="0 0 0"/></joint>
          <joint name="j3" type="fixed"><parent link="a"/><child link="c"/><origin xyz="0 1 0" rpy="0 0 0"/></joint>
          <joint name="j4" type="fixed"><parent link="c"/><child link="d"/><origin xyz="1 0 0" rpy="0 0 0"/></joint>
        </robot>""")
        val edges = Kinematics.buildEdges(GraphContext(r, "S1", emptyMap(), emptyList(), null))
        val paths = Kinematics.findPaths(edges, "a", "d")
        assertEquals(2, paths.size)
        assertTrue(paths.all { it.transform != null })
        assertTrue(paths.all { it.errorTranslation < 1e-9 })
        // both give translation (1,1,0)
        paths.forEach { p ->
            val m = p.transform!!.m
            assertEquals(1.0, m[3], 1e-9); assertEquals(1.0, m[7], 1e-9)
        }
    }

    @Test fun parallelInconsistentPathsKeepErrors() {
        val r = robot("""
        <robot name="dia2">
          <link name="a"/><link name="b"/><link name="c"/><link name="d"/>
          <joint name="j1" type="fixed"><parent link="a"/><child link="b"/><origin xyz="1 0 0" rpy="0 0 0"/></joint>
          <joint name="j2" type="fixed"><parent link="b"/><child link="d"/><origin xyz="0 1 0" rpy="0 0 0"/></joint>
          <joint name="j3" type="fixed"><parent link="a"/><child link="c"/><origin xyz="0 1 0" rpy="0 0 0"/></joint>
          <joint name="j4" type="fixed"><parent link="c"/><child link="d"/><origin xyz="1.1 0 0" rpy="0 0 0"/></joint>
        </robot>""")
        val edges = Kinematics.buildEdges(GraphContext(r, "S1", emptyMap(), emptyList(), null))
        val paths = Kinematics.findPaths(edges, "a", "d")
        assertEquals(2, paths.size)
        assertTrue(paths.all { it.errorTranslation > 0.09 }, "errors=${paths.map { it.errorTranslation }}")
    }
}

class UnitsTest {
    @Test fun degreeCalibrationConverted() {
        val cal = Calibration(1, "S1", "cam", "a", "b",
            doubleArrayOf(0.0, 0.0, 0.0), doubleArrayOf(0.0, 0.0, 90.0), "deg", null, null, 1)
        val t = cal.transform()
        // Rz(90deg): x-axis maps to y-axis
        assertEquals(0.0, t.m[0], 1e-12)
        assertEquals(1.0, t.m[4], 1e-12)
    }

    @Test fun prismaticUsesMeters() {
        val r = robot("""
        <robot name="p"><link name="a"/><link name="b"/>
          <joint name="jp" type="prismatic"><parent link="a"/><child link="b"/><axis xyz="1 0 0"/></joint>
        </robot>""")
        val edges = Kinematics.buildEdges(GraphContext(r, "S1", mapOf("jp" to 0.25), emptyList(), null))
        val e = edges.single()
        assertEquals("m", e.valueUnit)
        assertEquals(0.25, e.transform!!.m[3], 1e-12)
    }
}

class TimeValidityTest {
    private val cal = Calibration(1, "S1", "c", "a", "b",
        doubleArrayOf(0.0, 0.0, 0.0), doubleArrayOf(0.0, 0.0, 0.0), "rad", 1000L, 2000L, 1)

    @Test fun endpointsInclusive() {
        assertEquals("active:boundary-inclusive", cal.activeAt(1000L))
        assertEquals("active:boundary-inclusive", cal.activeAt(2000L))
        assertEquals("active", cal.activeAt(1500L))
    }

    @Test fun outsideRangeInactive() {
        assertEquals("inactive:before-valid-from", cal.activeAt(999L))
        assertEquals("inactive:after-valid-to", cal.activeAt(2001L))
        assertEquals("inactive:no-query-time", cal.activeAt(null))
    }

    @Test fun inactiveCalibrationExcludedFromGraph() {
        val r = robot("""<robot name="t"><link name="a"/><link name="b"/></robot>""")
        val edges = Kinematics.buildEdges(GraphContext(r, "S1", emptyMap(), listOf(cal), 999L))
        assertTrue(edges.isEmpty())
        val edges2 = Kinematics.buildEdges(GraphContext(r, "S1", emptyMap(), listOf(cal), 1000L))
        assertEquals(1, edges2.size)
        assertEquals("active:boundary-inclusive", edges2[0].state)
    }

    @Test fun serialMismatchExcluded() {
        val r = robot("""<robot name="t"><link name="a"/><link name="b"/></robot>""")
        val edges = Kinematics.buildEdges(GraphContext(r, "OTHER", emptyMap(), listOf(cal), 1500L))
        assertTrue(edges.isEmpty())
    }
}

class MissingJointValueTest {
    @Test fun missingValueMarkedAndPathIncomplete() {
        val r = robot(CHAIN)
        val edges = Kinematics.buildEdges(GraphContext(r, "S1", mapOf("j1" to 0.5), emptyList(), null))
        val j2 = edges.first { it.jointName == "j2" }
        assertEquals("missing_joint_value", j2.state)
        assertEquals(null, j2.transform)
        val paths = Kinematics.findPaths(edges, "a", "c")
        assertEquals(1, paths.size)
        assertTrue(paths[0].state.startsWith("incomplete"))
        assertEquals(null, paths[0].transform)
    }

    @Test fun cycleWithMissingEdgeIndeterminate() {
        val r = robot(CHAIN)
        val cal = Calibration(1, "S1", "loop", "c", "a",
            doubleArrayOf(-2.0, 0.0, 0.0), doubleArrayOf(0.0, 0.0, 0.0), "rad", null, null, 1)
        val edges = Kinematics.buildEdges(GraphContext(r, "S1", emptyMap(), listOf(cal), null))
        val cycles = Kinematics.cycleResiduals(edges)
        assertEquals(1, cycles.size)
        assertTrue(cycles[0].state.startsWith("indeterminate"))
    }
}

class XmlRoundTripTest {
    @Test fun unknownContentAndOrderPreserved() {
        val xml = """<?xml version="1.0"?>
<robot name="x" weird:attr="1" xmlns:weird="http://x">
  <gazebo reference="a"><material>Grey</material></gazebo>
  <link name="a"><custom foo="bar"/></link>
  <joint name="j1" type="fixed"><parent link="a"/><child link="b"/><origin xyz="0 0 1" rpy="0 0 0"/></joint>
  <link name="b"/>
  <transmission name="t1"><type>Simple</type></transmission>
</robot>"""
        val r = robot(xml)
        val out = UrdfParser.serialize(r.document)
        assertTrue(out.contains("gazebo"), out)
        assertTrue(out.contains("custom"), out)
        assertTrue(out.contains("transmission"), out)
        assertTrue(out.contains("weird:attr"))
        // order: gazebo before link a before joint before link b before transmission
        val order = listOf("gazebo", "custom", "<joint", "transmission").map { out.indexOf(it) }
        assertTrue(order.zipWithNext().all { (x, y) -> x in 0 until y }, "$order in $out")
    }

    @Test fun exportAppliesOnlyApprovedPatches() {
        val db = Db(":memory:")
        val xml = """<robot name="e"><link name="a"/><link name="b"/>
            <joint name="j1" type="fixed"><parent link="a"/><child link="b"/><origin xyz="0 0 1" rpy="0 0 0"/></joint>
            <unknown keep="yes"/></robot>""".trimIndent()
        val id = db.insertUrdf("e", "S1", 1, xml)
        val pending = db.insertDraft(id, 1, """{"ops":[{"op":"set_joint_origin","joint":"j1","xyz":[0,0,2],"rpy":[0,0,0]}]}""")
        // pending draft must NOT be applied
        var doc = db.urdfById(id)!!
        var parsed = UrdfParser.parse(doc.xml)
        Patches.applyAll(parsed.document, db.drafts().filter { it.status == "approved" }.map { it.patchJson })
        var out = UrdfParser.serialize(parsed.document)
        assertTrue(out.contains("xyz=\"0 0 1\""))
        // approve -> applied
        assertEquals("approved", db.approveDraft(pending))
        parsed = UrdfParser.parse(doc.xml)
        Patches.applyAll(parsed.document, db.drafts().filter { it.status == "approved" }.map { it.patchJson })
        out = UrdfParser.serialize(parsed.document)
        assertTrue(out.contains("xyz=\"0 0 2\""), out)
        assertTrue(out.contains("unknown"), out) // unknown content survives
    }

    @Test fun staleDraftRejected() {
        val db = Db(":memory:")
        val id1 = db.insertUrdf("e", "S1", 1, "<robot name=\"e\"/>")
        val draftId = db.insertDraft(id1, 1, """{"ops":[]}""")
        db.insertUrdf("e", "S1", 2, "<robot name=\"e\"/>") // concurrent edit bumps version
        assertEquals("conflict", db.approveDraft(draftId))
    }
}

class TransformTest {
    @Test fun composeInverseRoundTrip() {
        val t = Transform.fromXyzRpy(doubleArrayOf(1.0, 2.0, 3.0), doubleArrayOf(0.3, -0.2, 1.0))
        assertTrue(t.compose(t.inverse()).approxEquals(Transform.IDENTITY, 1e-9))
    }

    @Test fun rotationAboutZ() {
        val t = Transform.rotationAboutAxis(doubleArrayOf(0.0, 0.0, 1.0), PI / 2)
        assertEquals(0.0, t.m[0], 1e-12)
        assertEquals(1.0, t.m[4], 1e-12)
        assertEquals(PI / 2, t.rotationAngle(), 1e-9)
    }
}
