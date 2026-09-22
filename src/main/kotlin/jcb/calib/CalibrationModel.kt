package jcb.calib

import jcb.kin.EdgeSource
import jcb.kin.FrameEdge
import jcb.kin.Transform
import java.time.Instant

/**
 * 独立版本化标定变换。标定永远不写回原 URDF；
 * 仅在机器人序列号匹配、且时间落在 [validFrom, validTo) 时生效（左闭右开）。
 */
data class CalibrationTransform(
    val id: String,
    val robotSerial: String,
    val version: Int,
    val parentFrame: String,
    val childFrame: String,
    val xyz: DoubleArray,
    val rpyRad: DoubleArray,
    val note: String?,
    /** 生效起点（含），null 表示从无限早 */
    val validFrom: Instant?,
    /** 生效终点（不含），null 表示到无限远 */
    val validTo: Instant?,
    val createdAt: Instant,
    /** 草案编辑乐观锁基准：approved 后版本号 +1 */
    val status: CalibStatus,
    /** 草稿指向的上一个已批准版本（0 表示新建） */
    val baseVersion: Int,
) {
    override fun equals(other: Any?): Boolean = other is CalibrationTransform && id == other.id && version == other.version
    override fun hashCode(): Int = 31 * id.hashCode() + version
}

enum class CalibStatus { DRAFT, APPROVED, SUPERSEDED, REJECTED }

enum class EffectiveState {
    ACTIVE,            // 序列号匹配且在生效区间
    WRONG_ROBOT,       // 序列号不匹配
    BEFORE_WINDOW,     // 时间早于 validFrom（边界 validFrom 本身为 ACTIVE）
    AFTER_WINDOW,      // 时间 >= validTo（边界 validTo 不属于区间）
    DRAFT_NOT_APPROVED,
    INACTIVE_VERSION,  // 已被取代/驳回
    NO_SNAPSHOT_TIME,  // 快照无时间戳
}

data class EffectiveCalibration(
    val transform: CalibrationTransform,
    val state: EffectiveState,
    val effective: Boolean,
    val reason: String,
)

object CalibrationService {
    fun evaluate(
        calibs: List<CalibrationTransform>,
        robotSerial: String,
        at: Instant?,
    ): List<EffectiveCalibration> = calibs.map { c ->
        when {
            c.status == CalibStatus.DRAFT -> EffectiveCalibration(
                c, EffectiveState.DRAFT_NOT_APPROVED, false,
                "版本 v${c.version} 为草案，未获批不生效"
            )
            c.status != CalibStatus.APPROVED -> EffectiveCalibration(
                c, EffectiveState.INACTIVE_VERSION, false,
                "版本 v${c.version} 状态为 ${c.status}，不是当前有效版本"
            )
            c.robotSerial != robotSerial -> EffectiveCalibration(
                c, EffectiveState.WRONG_ROBOT, false,
                "标定绑定序列号 ${c.robotSerial}，当前机器人 $robotSerial"
            )
            at == null -> EffectiveCalibration(
                c, EffectiveState.NO_SNAPSHOT_TIME, false,
                "现场快照没有时间戳，无法判定时间生效区间"
            )
            c.validFrom != null && at.isBefore(c.validFrom) -> EffectiveCalibration(
                c, EffectiveState.BEFORE_WINDOW, false,
                "查询时刻 $at 早于生效起点 ${c.validFrom}（起点含）"
            )
            c.validTo != null && !at.isBefore(c.validTo) -> EffectiveCalibration(
                c, EffectiveState.AFTER_WINDOW, false,
                "查询时刻 $at 已到/超过生效终点 ${c.validTo}（终点不含，边界状态）"
            )
            else -> EffectiveCalibration(
                c, EffectiveState.ACTIVE, true,
                "序列号匹配且 $at 位于 [${c.validFrom ?: "-∞"}, ${c.validTo ?: "+∞"})"
            )
        }
    }.sortedWith(compareBy({ it.transform.parentFrame }, { it.transform.childFrame }, { it.transform.version }))

    /** 同一边的多个生效标定也算“多路径候选”，但同序列号下通常只应一个版本生效。 */
    fun toEdges(effective: List<EffectiveCalibration>): List<FrameEdge> =
        effective.filter { it.effective }.mapIndexed { index, ec ->
            val c = ec.transform
            FrameEdge(
                parent = c.parentFrame,
                child = c.childFrame,
                source = EdgeSource.Calibration(
                    c.id, "标定 ${c.id} v${c.version}（${c.robotSerial}）"
                ),
                baseTransform = Transform.xyzRpy(c.xyz, c.rpyRad),
                joint = null,
                fixed = true,
                declarationOrder = 10_000 + c.version * 100 + index,
            )
        }

    /** 版本差异：对同 parent/child 的标定版本逐字段对比。 */
    fun diff(older: List<CalibrationTransform>, newer: List<CalibrationTransform>): List<CalibDiffEntry> {
        val key = { c: CalibrationTransform -> c.parentFrame to c.childFrame }
        val oldByKey = older.groupBy(key)
        val newByKey = newer.groupBy(key)
        val keys = (oldByKey.keys + newByKey.keys).sortedWith(compareBy({ it.first }, { it.second }))
        return keys.flatMap { k ->
            val olds = oldByKey[k].orEmpty().sortedBy { it.version }
            val news = newByKey[k].orEmpty().sortedBy { it.version }
            when {
                olds.isEmpty() -> news.map {
                    CalibDiffEntry(k.first, k.second, null, it.version, listOf("新增标定边"))
                }
                news.isEmpty() -> olds.map {
                    CalibDiffEntry(k.first, k.second, it.version, null, listOf("标定边删除/失效"))
                }
                else -> {
                    val o = olds.last(); val nw = news.last()
                    val changes = mutableListOf<String>()
                    if (!o.xyz.contentEquals(nw.xyz))
                        changes += "平移: ${vec(o.xyz)} -> ${vec(nw.xyz)}"
                    if (!o.rpyRad.contentEquals(nw.rpyRad))
                        changes += "旋转(rad): ${vec(o.rpyRad)} -> ${vec(nw.rpyRad)}"
                    if (o.validFrom != nw.validFrom || o.validTo != nw.validTo)
                        changes += "生效区间: [${o.validFrom}, ${o.validTo}) -> [${nw.validFrom}, ${nw.validTo})"
                    if (o.robotSerial != nw.robotSerial)
                        changes += "序列号: ${o.robotSerial} -> ${nw.robotSerial}"
                    if (changes.isEmpty()) changes += "无几何/时间变化（仅版本或状态变化）"
                    listOf(CalibDiffEntry(k.first, k.second, o.version, nw.version, changes))
                }
            }
        }
    }

    private fun vec(a: DoubleArray) =
        a.joinToString(prefix = "[", postfix = "]") { "%.6f".format(it) }
}

data class CalibDiffEntry(
    val parent: String,
    val child: String,
    val oldVersion: Int?,
    val newVersion: Int?,
    val changes: List<String>,
)
