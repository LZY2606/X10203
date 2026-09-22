package jcb

import jcb.calib.CalibStatus
import jcb.calib.CalibrationService
import jcb.calib.CalibrationTransform
import jcb.calib.EffectiveState
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Test
import java.time.Instant

class CalibrationTimeTest {
    private val c = CalibrationTransform(
        id = "c", version = 1, robotSerial = "SN-1",
        parentFrame = "a", childFrame = "b",
        xyz = DoubleArray(3), rpyRad = DoubleArray(3),
        note = null,
        validFrom = Instant.parse("2026-09-01T00:00:00Z"),
        validTo = Instant.parse("2026-10-01T00:00:00Z"),
        createdAt = Instant.now(),
        status = CalibStatus.APPROVED, baseVersion = 0,
    )

    private fun state(at: String?, serial: String = "SN-1"): EffectiveState =
        CalibrationService.evaluate(listOf(c), serial, at?.let(Instant::parse)).single().state

    @Test
    fun `生效区间左闭右开`() {
        assertEquals(EffectiveState.BEFORE_WINDOW, state("2026-08-31T23:59:59Z"))
        assertEquals(EffectiveState.ACTIVE, state("2026-09-01T00:00:00Z"))   // 起点含
        assertEquals(EffectiveState.ACTIVE, state("2026-09-15T12:00:00Z"))
        assertEquals(EffectiveState.AFTER_WINDOW, state("2026-10-01T00:00:00Z")) // 终点不含
    }

    @Test
    fun `序列号不匹配不生效`() {
        assertEquals(EffectiveState.WRONG_ROBOT, state("2026-09-15T00:00:00Z", "SN-OTHER"))
    }

    @Test
    fun `无时间戳状态明确`() {
        assertEquals(EffectiveState.NO_SNAPSHOT_TIME,
            CalibrationService.evaluate(listOf(c), "SN-1", null).single().state)
    }

    @Test
    fun `草案与失效版本不生效`() {
        assertEquals(EffectiveState.DRAFT_NOT_APPROVED,
            CalibrationService.evaluate(listOf(c.copy(status = CalibStatus.DRAFT)),
                "SN-1", Instant.parse("2026-09-15T00:00:00Z")).single().state)
        assertEquals(EffectiveState.INACTIVE_VERSION,
            CalibrationService.evaluate(listOf(c.copy(status = CalibStatus.SUPERSEDED)),
                "SN-1", Instant.parse("2026-09-15T00:00:00Z")).single().state)
    }
}
