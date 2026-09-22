package atlas

import atlas.kinematics.Engine
import atlas.urdf.UrdfParser
import kotlin.math.abs
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue

class MimicTest {
    private fun robot(j2mimic: String, j3mimic: String) = UrdfParser.parse(
        """<robot name="m">
          <link name="a"/><link name="b"/><link name="c"/><link name="d"/>
          <joint name="j1" type="revolute"><parent link="a"/><child link="b"/><axis xyz="0 0 1"/></joint>
          <joint name="j2" type="revolute"><parent link="b"/><child link="c"/><axis xyz="0 0 1"/>$j2mimic</joint>
          <joint name="j3" type="revolute"><parent link="c"/><child link="d"/><axis xyz="0 0 1"/>$j3mimic</joint>
        </robot>"""
    )

    @Test
    fun `mimic chain propagates multiplier and offset`() {
        val r = robot(
            """<mimic joint="j1" multiplier="2.0" offset="0.1"/>""",
            """<mimic joint="j2" multiplier="0.5" offset="0.0"/>"""
        )
        val e = Engine(r, emptyList(), mapOf("j1" to 0.4))
        // j2 = 2*0.4+0.1 = 0.9 ; j3 = 0.5*0.9 = 0.45
        assertEquals(0.9, e.resolveJoint("j2").value, 1e-9)
        assertEquals("MIMIC", e.resolveJoint("j2").state)
        assertEquals(0.45, e.resolveJoint("j3").value, 1e-9)
        // total rotation a->d = 0.4 + 0.9 + 0.45 = 1.75 rad about z
        val q = e.query("a", "d")
        assertEquals("OK", q.status)
        val m = q.candidates.single().matrix
        val expected = 1.75
        assertTrue(abs(m[0] - kotlin.math.cos(expected)) < 1e-9)
        assertTrue(abs(m[1] + kotlin.math.sin(expected)) < 1e-9)
    }

    @Test
    fun `mimic cycle is reported not unfolded`() {
        val r = robot(
            """<mimic joint="j3" multiplier="1.0"/>""",
            """<mimic joint="j2" multiplier="1.0"/>"""
        )
        val e = Engine(r, emptyList(), mapOf("j1" to 0.1))
        assertEquals("MIMIC_CYCLE", e.resolveJoint("j2").state)
        assertEquals("MIMIC_CYCLE", e.resolveJoint("j3").state)
        val q = e.query("a", "d")
        assertEquals("MIMIC_CYCLE", q.status)
        assertTrue(q.issues.any { it.startsWith("MIMIC_CYCLE") })
    }
}
