package atlas

import atlas.kinematics.Engine
import atlas.urdf.UrdfParser
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue

class CycleTest {
    /** Square loop a->b->c->d->a of fixed joints. */
    private fun loopRobot(dzLast: Double) = UrdfParser.parse(
        """<robot name="loop">
          <link name="a"/><link name="b"/><link name="c"/><link name="d"/>
          <joint name="ab" type="fixed"><parent link="a"/><child link="b"/><origin xyz="1 0 0"/></joint>
          <joint name="bc" type="fixed"><parent link="b"/><child link="c"/><origin xyz="0 1 0"/></joint>
          <joint name="cd" type="fixed"><parent link="c"/><child link="d"/><origin xyz="-1 0 0"/></joint>
          <joint name="da" type="fixed"><parent link="d"/><child link="a"/><origin xyz="0 -1 $dzLast"/></joint>
        </robot>"""
    )

    @Test
    fun `consistent loop has zero residual and no conflict edges`() {
        val e = Engine(loopRobot(0.0), emptyList(), emptyMap())
        val report = e.cycleReport()
        assertEquals(1, report.cycles.size)
        assertTrue(report.cycles[0].residual < 1e-9, "residual=${report.cycles[0].residual}")
        assertTrue(report.cycles[0].consistent)
        assertTrue(report.minimalConflictEdgeSets.isEmpty())
    }

    @Test
    fun `inconsistent loop reports residual and minimal conflict edge set`() {
        val e = Engine(loopRobot(0.5), emptyList(), emptyMap())
        val report = e.cycleReport()
        assertEquals(1, report.cycles.size)
        assertTrue(report.cycles[0].residual > 0.4)
        assertTrue(!report.cycles[0].consistent)
        // removing any single edge breaks the loop; the minimal set must contain exactly one edge
        assertTrue(report.minimalConflictEdgeSets.isNotEmpty())
        assertTrue(report.minimalConflictEdgeSets.all { it.size == 1 })
        val edgeIds = report.minimalConflictEdgeSets.flatten().toSet()
        assertTrue(edgeIds.all { it.startsWith("urdf:joint:") })
    }

    @Test
    fun `cycle is kept as cycle not unfolded into tree`() {
        val e = Engine(loopRobot(0.0), emptyList(), emptyMap())
        // graph still has 4 edges and 4 frames: the loop edge was not dropped
        assertEquals(4, e.edges.size)
        assertEquals(4, e.frames().size)
        // query across the loop yields multiple candidate paths
        val q = e.query("a", "c")
        assertEquals(2, q.candidates.size)
        assertTrue(q.candidates.all { it.consistentWithOthers })
    }
}
