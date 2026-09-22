package book

import book.app.AppService
import book.calib.CalibRules
import book.calib.CalibrationVersion
import book.db.Database
import org.junit.jupiter.api.io.TempDir
import java.io.File
import java.nio.file.Path
import java.time.Instant
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFailsWith
import kotlin.test.assertTrue

class CalibrationServiceTest {

    @TempDir
    lateinit var tempDir: Path

    private fun service(): Pair<AppService, File> {
        val dbFile = tempDir.resolve("test.sqlite").toFile()
        val db = Database(dbFile.absolutePath)
        val svc = AppService(db)
        db.upsertRobot("SN-9", "测试机", """
            <?xml version="1.0"?><robot name="t">
            <link name="a"/><link name="b"/>
            <joint name="j1" type="fixed"><parent link="a"/><child link="b"/>
            <origin xyz="0 0 0" rpy="0 0 0"/></joint>
            </robot>
        """.trimIndent())
        return svc to dbFile
    }

    private fun windowVersion(from: String, until: String, serial: String = "SN-9") =
        CalibrationVersion(
            id = "v", robotSerial = serial, version = 1, note = "",
            createdAt = "2026-01-01T00:00:00Z",
            validFrom = from, validUntil = until,
            draft = false, approved = true, edges = emptyList(),
        )

    @Test
    fun `time window endpoints are explicit states`() {
        val from = Instant.parse("2026-01-01T00:00:00Z")
        val until = Instant.parse("2026-01-02T00:00:00Z")
        val v = windowVersion(from.toString(), until.toString())
        assertEquals(CalibValidityStatusName.BOUNDARY_START,
            name(CalibRules.evaluate(v, "SN-9", from)))
        assertEquals(CalibValidityStatusName.EXPIRED,
            name(CalibRules.evaluate(v, "SN-9", until)))
        assertEquals(CalibValidityStatusName.ACTIVE,
            name(CalibRules.evaluate(v, "SN-9", from.plusSeconds(60))))
        assertEquals(CalibValidityStatusName.FUTURE,
            name(CalibRules.evaluate(v, "SN-9", from.minusSeconds(60))))
    }

    @Test
    fun `serial mismatch disables calibration`() {
        val v = windowVersion("2026-01-01T00:00:00Z", "2030-01-01T00:00:00Z", serial = "OTHER")
        assertEquals(CalibValidityStatusName.SERIAL_MISMATCH,
            name(CalibRules.evaluate(v, "SN-9", Instant.parse("2026-02-01T00:00:00Z"))))
    }

    @Test
    fun `optimistic version conflict is rejected`() {
        val (svc, _) = service()
        val draft = svc.createDraft(
            "SN-9", "草案", "2026-01-01T00:00:00Z", "2030-01-01T00:00:00Z", emptyList())
        svc.updateDraft(draft.id, 0, "改 A", false, emptyList())
        // 并发方仍持 lock_version=0，第二次编辑应被拒绝
        assertFailsWith<java.util.ConcurrentModificationException> {
            svc.updateDraft(draft.id, 0, "改 B（冲突）", false, emptyList())
        }
        // 持新 lock_version=1 可继续编辑
        svc.updateDraft(draft.id, 1, "改 C（成功）", false, emptyList())
    }

    @Test
    fun `frozen snapshot can be evaluated against another calibration with validity status`() {
        val (svc, _) = service()
        val active = svc.createDraft(
            "SN-9", "有效标定", "2000-01-01T00:00:00Z", "2030-01-01T00:00:00Z", emptyList())
        svc.approveVersion(active.id)
        val future = svc.createDraft(
            "SN-9", "未来标定", "2099-01-01T00:00:00Z", "2099-06-01T00:00:00Z", emptyList())
        svc.approveVersion(future.id)

        val snap = svc.freezeSnapshot(
            "SN-9", "s1", emptyMap(), emptyMap(), active.id, "note")
        val cmp = svc.compareSnapshot(snap.id, future.id, listOf("a" to "b"))
        assertTrue(cmp.validity.startsWith("FUTURE"), cmp.validity)
        assertTrue(cmp.otherCalib!!.validityStatus == "FUTURE")
    }

    private enum class CalibValidityStatusName {
        ACTIVE, FUTURE, EXPIRED, BOUNDARY_START, BOUNDARY_END, SERIAL_MISMATCH
    }
    private fun name(v: book.calib.CalibValidity) =
        CalibValidityStatusName.valueOf(v.status.name)
}
