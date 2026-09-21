package coordbook

import java.nio.file.Files
import kotlin.test.AfterTest
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFailsWith

class DatabaseTest {

    private val tempDb = Files.createTempFile("coordbook-test", ".db").also { it.toFile().deleteOnExit() }
    private val db = Database(tempDb.toString())

    @AfterTest
    fun close() {
        db.conn.close()
    }

    private val urdf = """<?xml version="1.0"?>
<robot name="t"><link name="a"/></robot>
"""

    @Test
    fun testOptimisticConcurrencyOnUrdfRevision() {
        val robot = db.insertRobot("t", "SN-DB", urdf)
        val updated = db.updateUrdf(robot.id, urdf.replace("t", "t2"), robot.currentRevision)
        assertEquals(robot.currentRevision + 1, updated.currentRevision)
        assertFailsWith<OptimisticLockException> {
            db.updateUrdf(robot.id, urdf, robot.currentRevision) // 过期 revision
        }
    }

    @Test
    fun testPatchDraftStatusConflict() {
        val robot = db.insertRobot("t", "SN-DB2", urdf)
        val v = db.insertCalibVersion(
            robot.id, "SN-DB2",
            "2026-09-01T00:00:00Z", "2026-12-31T23:59:59Z", null,
        )
        val patch = db.insertPatch(
            v.id, PatchKind.JOINT_ORIGIN_RPY, "j", null, null,
            null, doubleArrayOf(0.0, 0.0, 0.1), null,
        )
        val approved = db.setPatchStatus(patch.id, PatchStatus.APPROVED, v.revision)
        assertEquals(PatchStatus.APPROVED, approved.status)
        assertFailsWith<OptimisticLockException> {
            db.setPatchStatus(patch.id, PatchStatus.REJECTED, v.revision + 1)
        }
    }

    @Test
    fun testSnapshotPersistsJointValues() {
        val robot = db.insertRobot("t", "SN-DB3", urdf)
        val snap = db.insertSnapshot(
            robot.id, "SN-DB3", "2026-09-21T08:00:00Z",
            JsonMaps.encodeDoubles(mapOf("j1" to 1.5)), null,
        )
        assertEquals(1.5, db.getSnapshot(snap.id)!!.jointValues["j1"])
    }
}
