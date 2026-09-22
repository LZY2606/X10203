package atlas

import atlas.kinematics.CalibrationSet
import atlas.kinematics.CalibrationTransform
import atlas.kinematics.Engine
import atlas.urdf.UrdfParser
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue

class MultiPathTest {
    private val robot = UrdfParser.parse(
        """<robot name="mp">
          <link name="a"/><link name="b"/><link name="c"/>
          <joint name="ab" type="fixed"><parent link="a"/><child link="b"/><origin xyz="1 0 0"/></joint>
          <joint name="bc" type="fixed"><parent link="b"/><child link="c"/><origin xyz="0 1 0"/></joint>
        </robot>"""
    )

    private fun cal(x: Double) = CalibrationSet(
        id = 7, name = "cal", transforms = listOf(
            CalibrationTransform("a", "c", listOf(x, 1.0, 0.0), listOf(0.0, 0.0, 0.0))
        )
    )

    @Test
    fun `duplicate urdf and calibration declaration both kept as candidates`() {
        val e = Engine(robot, listOf(cal(1.0)), emptyMap())
        val q = e.query("a", "c")
        assertEquals("OK", q.status)
        // path 1: a->b->c via URDF; path 2: a->c via calibration
        assertEquals(2, q.candidates.size)
        val sources = q.candidates.map { cand -> cand.edges.map { it.source }.toSet() }
        assertTrue(sources.any { it == setOf("urdf") })
        assertTrue(sources.any { it == setOf("calibration") })
        // consistent: calibration matches composed URDF transform
        assertTrue(q.candidates.all { it.error < 1e-9 })
        // no direct duplicate pair here: URDF reaches c via b, calibration is direct
        assertTrue(e.edges.none { it.duplicateDeclaration })
    }

    @Test
    fun `same transform declared by urdf and calibration is flagged duplicate`() {
        val direct = UrdfParser.parse(
            """<robot name="dd"><link name="a"/><link name="b"/>
              <joint name="ab" type="fixed"><parent link="a"/><child link="b"/><origin xyz="1 0 0"/></joint></robot>"""
        )
        val cal = CalibrationSet(
            id = 9, name = "dup", transforms = listOf(
                CalibrationTransform("a", "b", listOf(1.0, 0.0, 0.0), listOf(0.0, 0.0, 0.0))
            )
        )
        val e = Engine(direct, listOf(cal), emptyMap())
        val dup = e.edges.filter { it.duplicateDeclaration }
        assertEquals(2, dup.size)
        assertEquals(setOf("urdf", "calibration"), dup.map { it.source }.toSet())
        val q = e.query("a", "b")
        assertEquals(2, q.candidates.size)
        assertTrue(q.candidates.all { it.error < 1e-9 })
    }

    @Test
    fun `conflicting paths keep error instead of picking traversal winner`() {
        val e = Engine(robot, listOf(cal(2.0)), emptyMap())
        val q = e.query("a", "c")
        assertEquals(2, q.candidates.size)
        assertTrue(q.candidates.all { it.error > 0.9 }, "errors=${q.candidates.map { it.error }}")
        assertTrue(q.candidates.none { it.consistentWithOthers })
        assertTrue(q.issues.any { it.startsWith("MULTIPLE_PATHS") })
    }

    @Test
    fun `candidate set does not depend on edge insertion order`() {
        val reversed = CalibrationSet(
            id = 8, name = "cal2", transforms = listOf(
                CalibrationTransform("a", "c", listOf(1.0, 1.0, 0.0), listOf(0.0, 0.0, 0.0))
            )
        )
        val q1 = Engine(robot, listOf(cal(1.0)), emptyMap()).query("a", "c")
        val q2 = Engine(robot, listOf(reversed), emptyMap()).query("a", "c")
        val mats1 = q1.candidates.map { it.matrix }.sortedBy { it.hashCode() }
        val mats2 = q2.candidates.map { it.matrix }.sortedBy { it.hashCode() }
        assertEquals(mats1.size, mats2.size)
        for (m in mats1) assertTrue(mats2.any { n -> atlas.kinematics.Mat.maxDiff(m.toDoubleArray(), n.toDoubleArray()) < 1e-12 })
    }
}
