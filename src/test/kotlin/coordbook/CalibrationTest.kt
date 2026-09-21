package coordbook

import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue

class CalibrationTest {

    private val version = CalibrationVersion(
        id = 1, robotId = 1, serialScope = "SN-1",
        validFromIso = "2026-09-01T00:00:00Z",
        validUntilIso = "2026-09-30T23:59:59Z",
        revision = 1, note = null, createdAt = "",
    )

    @Test
    fun testTimeWindowEndpointsAreEffectiveInclusive() {
        assertEquals(
            EffectiveState.EFFECTIVE,
            CalibrationService.effectiveAt(version, "SN-1", "2026-09-01T00:00:00Z").state,
        )
        assertEquals(
            EffectiveState.EFFECTIVE,
            CalibrationService.effectiveAt(version, "SN-1", "2026-09-30T23:59:59Z").state,
        )
    }

    @Test
    fun testBeforeAndAfterWindowAreIneffective() {
        assertEquals(
            EffectiveState.OUT_OF_WINDOW,
            CalibrationService.effectiveAt(version, "SN-1", "2026-08-31T23:59:59Z").state,
        )
        assertEquals(
            EffectiveState.OUT_OF_WINDOW,
            CalibrationService.effectiveAt(version, "SN-1", "2026-10-01T00:00:00Z").state,
        )
    }

    @Test
    fun testSerialMismatch() {
        val r = CalibrationService.effectiveAt(version, "SN-OTHER", "2026-09-15T00:00:00Z")
        assertEquals(EffectiveState.SERIAL_MISMATCH, r.state)
    }

    @Test
    fun testExtraTransformBecomesParallelCandidateWithError() {
        val xml = """<?xml version="1.0"?>
<robot name="cal">
  <link name="a"/><link name="b"/>
  <joint name="j" type="fixed"><parent link="a"/><child link="b"/>
    <origin xyz="1 0 0" rpy="0 0 0"/></joint>
</robot>
"""
        val doc = UrdfParser.parse(xml)
        val (graph, _) = GraphBuilder.build(doc)
        val patch = CalibPatch(
            id = 9, versionId = 1, kind = PatchKind.EXTRA_TRANSFORM,
            targetJoint = null, parentFrame = "a", childFrame = "b",
            xyz = doubleArrayOf(1.02, 0.0, 0.0), rpyRadians = doubleArrayOf(0.0, 0.0, 0.0),
            status = PatchStatus.APPROVED, note = null, createdAt = "",
        )
        val issues = CalibrationService.applyApproved(graph, listOf(patch), 1, 1)
        assertTrue(issues.any { it.code == "URDF_CALIB_DUPLICATE" })
        val q = Kinematics.query(graph, doc, "a", "b", emptyMap()).report!!
        assertEquals(2, q.candidates.size)
        assertEquals(0.02, q.maxTranslationErrorM, 1e-9)
        // 边来源必须同时含 urdf 与 calibration
        val kinds = q.candidates.flatMap { c -> c.steps.map { it.edgeKind } }.toSet()
        assertEquals(setOf("urdf", "calibration"), kinds)
    }

    @Test
    fun testEffectiveVersionUsesSerialTimeAndApproval() {
        val v2 = version.copy(id = 2, revision = 2, validFromIso = "2026-10-01T00:00:00Z")
        val (chosen, result) = CalibrationService.selectEffective(
            listOf(version, v2),
            approval = { it == 1L },
            serial = "SN-1",
            atIso = "2026-09-15T12:00:00Z",
        )
        assertEquals(1L, chosen?.id)
        assertEquals(EffectiveState.EFFECTIVE, result.state)
        // v2 不在时间窗
        val (none2, state2) = CalibrationService.selectEffective(
            listOf(v2), approval = { true }, "SN-1", "2026-09-15T12:00:00Z",
        )
        assertEquals(null, none2)
        assertEquals(EffectiveState.OUT_OF_WINDOW, state2.state)
    }
}
