package atlas

import atlas.kinematics.CalibrationSet
import atlas.kinematics.CalibrationTransform
import atlas.store.PatchOp
import atlas.store.Snapshot
import atlas.store.Store
import atlas.urdf.PatchApplier
import atlas.urdf.UrdfDiff
import atlas.urdf.UrdfParser
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertTrue

class StoreDraftTest {
    private val xml = """<robot name="s">
  <link name="a"/><link name="b"/>
  <joint name="j1" type="revolute">
    <parent link="a"/><child link="b"/>
    <origin xyz="0 0 1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1" upper="1"/>
  </joint>
</robot>"""

    private fun store(): Store = Store(":memory:")

    @Test
    fun `approved draft applies with version bump, stale base conflicts`() {
        store().use { s ->
            val doc = s.insertUrdf("s", xml)
            val d1 = s.insertDraft(doc.id, 1, listOf(PatchOp("setOrigin", "j1", xyz = listOf(0.0, 0.0, 2.0))))
            val d2 = s.insertDraft(doc.id, 1, listOf(PatchOp("setOrigin", "j1", xyz = listOf(0.0, 0.0, 3.0))))

            // unapproved draft cannot be applied
            val (ok0, _) = s.applyDraft(d1, xml)
            assertFalse(ok0)

            s.setDraftStatus(d1, "APPROVED")
            s.setDraftStatus(d2, "APPROVED")

            val newXml1 = PatchApplier.apply(UrdfParser.parse(xml).document, s.getDraft(d1)!!.ops).serialize()
            val (ok1, msg1) = s.applyDraft(d1, newXml1)
            assertTrue(ok1, msg1)
            assertEquals(2, s.getUrdf(doc.id)!!.version)

            // d2 was based on version 1 which is now stale -> conflict
            val newXml2 = PatchApplier.apply(UrdfParser.parse(xml).document, s.getDraft(d2)!!.ops).serialize()
            val (ok2, msg2) = s.applyDraft(d2, newXml2)
            assertFalse(ok2)
            assertTrue(msg2.contains("conflict"), msg2)
            assertEquals(2, s.getUrdf(doc.id)!!.version)

            // exported xml contains the approved patch only
            val exported = s.getUrdf(doc.id)!!.xml
            assertTrue(exported.contains("xyz=\"0.0 0.0 2.0\""), exported)
            assertFalse(exported.contains("3.0"))
        }
    }

    @Test
    fun `version history diff reports joint origin change`() {
        store().use { s ->
            val doc = s.insertUrdf("s", xml)
            val d = s.insertDraft(doc.id, 1, listOf(PatchOp("setLimit", "j1", lower = -2.0, upper = 2.0)))
            s.setDraftStatus(d, "APPROVED")
            val newXml = PatchApplier.apply(UrdfParser.parse(xml).document, s.getDraft(d)!!.ops).serialize()
            s.applyDraft(d, newXml)
            val diff = UrdfDiff.diff(s.urdfVersionXml(doc.id, 1)!!, s.urdfVersionXml(doc.id, 2)!!)
            assertTrue(diff.any { it.kind == "joint-limit" && it.name == "j1" }, diff.toString())
        }
    }

    @Test
    fun `calibration sets and snapshots persist independently of urdf`() {
        store().use { s ->
            val doc = s.insertUrdf("s", xml)
            val calId = s.insertCalibration(
                CalibrationSet(0, "cal", "SN-1", 100L, 200L,
                    transforms = listOf(CalibrationTransform("a", "b", listOf(1.0, 0.0, 0.0), listOf(0.0, 0.0, 0.0))))
            )
            val snapId = s.insertSnapshot(Snapshot(0, "snap", "SN-1", mapOf("j1" to 0.5), 150L, calId))
            val snap = s.getSnapshot(snapId)!!
            assertEquals(0.5, snap.jointValues["j1"])
            assertEquals(calId, snap.calibrationId)
            // urdf version untouched by calibration/snapshot writes
            assertEquals(1, s.getUrdf(doc.id)!!.version)
            val cal = s.getCalibration(calId)!!
            assertEquals(100L, cal.validFrom)
            assertEquals("b", cal.transforms[0].child)
        }
    }
}
