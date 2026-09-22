package atlas

import atlas.kinematics.CalibrationSet
import atlas.kinematics.CalibrationTransform
import atlas.kinematics.Engine
import atlas.server.JointAtlasServer
import atlas.urdf.UrdfParser
import kotlin.math.PI
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue

class UnitsAndStatusTest {
    private val robot = UrdfParser.parse(
        """<robot name="u">
          <link name="a"/><link name="b"/><link name="c"/>
          <joint name="j1" type="revolute"><parent link="a"/><child link="b"/><axis xyz="0 0 1"/></joint>
          <joint name="j2" type="fixed"><parent link="b"/><child link="c"/><origin xyz="0 0 1"/></joint>
        </robot>"""
    )

    @Test
    fun `angle units convert between rad and deg`() {
        val e = Engine(robot, emptyList(), mapOf("j1" to PI / 2))
        val rad = e.query("a", "c")
        assertEquals(PI / 2, rad.joints.first { it.joint == "j1" }.value, 1e-12)
        val deg = with(JointAtlasServer) { rad.withUnits("deg") }
        assertEquals(90.0, deg.joints.first { it.joint == "j1" }.value, 1e-9)
        assertEquals("deg", deg.units)
        val edge = deg.candidates.single().edges.first { it.jointName == "j1" }
        assertEquals(90.0, edge.jointValue!!, 1e-9)
        // matrix is unit-independent
        assertEquals(rad.candidates.single().matrix, deg.candidates.single().matrix)
    }

    @Test
    fun `missing joint value has explicit status`() {
        val e = Engine(robot, emptyList(), emptyMap())
        val q = e.query("a", "c")
        assertEquals("MISSING_JOINT_VALUE", q.status)
        assertTrue(q.issues.contains("MISSING_JOINT_VALUE:j1"))
        assertEquals("MISSING", q.joints.first { it.joint == "j1" }.state)
    }

    @Test
    fun `broken chain reports NO_PATH`() {
        val broken = UrdfParser.parse(
            """<robot name="b"><link name="a"/><link name="b"/><link name="island"/>
              <joint name="j1" type="fixed"><parent link="a"/><child link="b"/></joint></robot>"""
        )
        val e = Engine(broken, emptyList(), emptyMap())
        assertEquals("NO_PATH", e.query("a", "island").status)
        assertEquals("UNKNOWN_FRAME", e.query("a", "ghost").status)
    }

    @Test
    fun `duplicate frame names are flagged`() {
        val dup = UrdfParser.parse(
            """<robot name="d"><link name="a"/><link name="a"/><link name="b"/>
              <joint name="j1" type="fixed"><parent link="a"/><child link="b"/></joint></robot>"""
        )
        val e = Engine(dup, emptyList(), emptyMap())
        assertTrue(e.issues.contains("DUPLICATE_FRAME:a"))
        assertTrue(e.query("a", "b").issues.contains("DUPLICATE_FRAME:a"))
    }
}

class TimeValidityTest {
    private val robot = UrdfParser.parse(
        """<robot name="t"><link name="a"/><link name="b"/>
          <joint name="j1" type="fixed"><parent link="a"/><child link="b"/><origin xyz="1 0 0"/></joint></robot>"""
    )

    private fun cal() = CalibrationSet(
        id = 3, name = "timed", robotSerial = "SN-1", validFrom = 1000L, validTo = 2000L,
        transforms = listOf(CalibrationTransform("a", "b", listOf(1.0, 0.0, 0.0), listOf(0.0, 0.0, 0.0)))
    )

    @Test
    fun `boundary endpoints are active and flagged`() {
        for (t in listOf(1000L, 2000L)) {
            val e = Engine(robot, listOf(cal()), emptyMap(), serial = "SN-1", at = t)
            assertTrue(e.inactiveCalibrations.isEmpty(), "t=$t should be active")
            val edge = e.edges.first { it.source == "calibration" }
            assertTrue(edge.atValidityBoundary, "t=$t should be boundary")
        }
    }

    @Test
    fun `outside validity window is inactive with reason`() {
        val before = Engine(robot, listOf(cal()), emptyMap(), serial = "SN-1", at = 999L)
        assertEquals("BEFORE_VALID_FROM", before.inactiveCalibrations.single().reason)
        val after = Engine(robot, listOf(cal()), emptyMap(), serial = "SN-1", at = 2001L)
        assertEquals("AFTER_VALID_TO", after.inactiveCalibrations.single().reason)
        // inactive calibration contributes no edges
        assertTrue(before.edges.none { it.source == "calibration" })
    }

    @Test
    fun `serial mismatch disables calibration`() {
        val e = Engine(robot, listOf(cal()), emptyMap(), serial = "SN-2", at = 1500L)
        assertEquals("SERIAL_MISMATCH", e.inactiveCalibrations.single().reason)
        val ok = Engine(robot, listOf(cal()), emptyMap(), serial = "SN-1", at = 1500L)
        assertTrue(ok.inactiveCalibrations.isEmpty())
        assertEquals(2, ok.query("a", "b").candidates.size)
    }

    @Test
    fun `mid window is active without boundary flag`() {
        val e = Engine(robot, listOf(cal()), emptyMap(), serial = "SN-1", at = 1500L)
        val edge = e.edges.first { it.source == "calibration" }
        assertTrue(!edge.atValidityBoundary)
    }
}
