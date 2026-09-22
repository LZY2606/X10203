package jcb

import jcb.db.Database
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Assertions.assertThrows
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

class ServiceFlowTest {
    private val xml = """
<robot name="r" xmlns:jcb="http://x">
  <jcb:kept custom="1"/>
  <link name="base"/><link name="tip"/>
  <joint name="j1" type="fixed">
    <origin xyz="1 0 0" rpy="0 0 0"/>
    <parent link="base"/><child link="tip"/>
  </joint>
</robot>"""

    private fun newService() = AppService(Database())

    @Test
    fun `草案乐观锁并发冲突`() {
        val s = newService()
        s.importRobot("SN-1", null, xml)
        s.saveDraft("c1", "SN-1", 0, "base", "tip",
            listOf(1.0, 0.0, 0.0), listOf(0.0, 0.0, 0.0), null, null, null)
        val approved = s.approve("c1", 1)
        assertEquals(2, approved.version)
        // 仍基于 v0 的旧草案必须冲突
        assertThrows(VersionConflictException::class.java) {
            s.saveDraft("c1", "SN-1", 0, "base", "tip",
                listOf(1.01, 0.0, 0.0), listOf(0.0, 0.0, 0.0), null, null, null)
        }
        // 基于最新版本可以继续
        s.saveDraft("c1", "SN-1", 2, "base", "tip",
            listOf(1.02, 0.0, 0.0), listOf(0.0, 0.0, 0.0), null, null, null)
    }

    @Test
    fun `导出只应用获批补丁且保留未知内容并声明派生`() {
        val s = newService()
        s.importRobot("SN-1", null, xml)
        s.saveDraft("c1", "SN-1", 0, "base", "tip",
            listOf(1.05, 0.0, 0.0), listOf(0.0, 0.0, 0.0), "现场修正", null, null)
        // 未批准：导出不应包含修正
        val before = s.exportDerivedUrdf("SN-1", null)
        assertTrue(before.contains("jcb:kept"))
        assertFalse(before.contains("1.05"))
        assertTrue(before.contains("派生 URDF"))
        s.approve("c1", 1)
        val after = s.exportDerivedUrdf("SN-1", null)
        assertTrue(after.contains("jcb:kept"))
        assertTrue(after.contains("1.05"))
        assertTrue(after.contains("calibration:c1@v"))
        assertTrue(after.contains("原始 URDF 未被修改"))
    }

    @Test
    fun `快照冻结后可比较两套标定`() {
        val s = newService()
        s.importRobot("SN-1", null, xml)
        val snapId = s.createSnapshot(
            "SN-1", "冻结点", "2026-09-15T00:00:00Z", emptyMap()
        )
        s.freezeSnapshot(snapId)
        assertThrows(IllegalArgumentException::class.java) { s.freezeSnapshot(snapId) }

        s.saveDraft("c1", "SN-1", 0, "base", "tip",
            listOf(1.0, 0.0, 0.0), listOf(0.0, 0.0, 0.0), null,
            "2026-09-01T00:00:00Z", "2026-10-01T00:00:00Z")
        s.approve("c1", 1)
        s.saveDraft("c1", "SN-1", 2, "base", "tip",
            listOf(1.03, 0.0, 0.0), listOf(0.0, 0.0, 0.0), null,
            "2026-09-01T00:00:00Z", "2026-10-01T00:00:00Z")
        s.approve("c1", 3)

        val base = AppService.Scenario(
            "SN-1", snapId, "base", "tip", listOf("c1@2"), null
        )
        val other = base.copy(calibrationIds = listOf("c1@4"))
        val cmp = s.compare(base, other)
        println("DIFFS=${cmp.diffs.map { it.changes }}")
        assertTrue(cmp.diffs.any { it.changes.any { ch -> ch.contains("平移") } })
        // v2 在冻结时刻生效
        assertTrue(cmp.a.effective.single().effective)
        assertTrue(cmp.b.effective.single().effective)
    }

    @Test
    fun `时间窗外的标定快照查询显式版本时状态不生效`() {
        val s = newService()
        s.importRobot("SN-1", null, xml)
        val snapId = s.createSnapshot("SN-1", "早", "2026-01-01T00:00:00Z", emptyMap())
        s.saveDraft("c1", "SN-1", 0, "base", "tip",
            listOf(2.0, 0.0, 0.0), listOf(0.0, 0.0, 0.0), null,
            "2026-09-01T00:00:00Z", "2026-10-01T00:00:00Z")
        s.approve("c1", 1)
        val ev = s.evaluate(AppService.Scenario("SN-1", snapId, "base", "tip", listOf("c1@2"), null))
        val cal = ev.effective.single()
        assertFalse(cal.effective)
        assertEquals("BEFORE_WINDOW", cal.state.name)
        // 未生效标定不参与图：候选只走 URDF，x=1
        assertEquals(1.0, ev.query.candidates.single().transform[0, 3], 1e-9)
    }
}
