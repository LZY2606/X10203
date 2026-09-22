package jcb.calib

import jcb.model.CalibrationDeclaration
import jcb.model.CalibrationVersion
import java.time.Instant

data class CalibrationEffective(
    val versionId: String,
    val label: String,
    val declarations: List<CalibrationDeclaration>,
    val timeStatus: CalibrationTimeStatus,
)

enum class CalibrationTimeStatus {
    /** 时间严格落在生效区间内部。 */
    ACTIVE,
    /** 恰好等于 validFrom 或 validUntil（端点），按显式策略决定，默认仍生效但标记。 */
    AT_BOUNDARY,
    /** 早于生效起点。 */
    NOT_YET_VALID,
    /** 晚于生效终点。 */
    EXPIRED,
    /** 该序列号不匹配。 */
    SERIAL_MISMATCH,
    /** 未设置任何时间窗（始终生效）。 */
    UNBOUNDED,
    /** 现场查询时间缺失，无法判定。 */
    TIME_MISSING,
}

/**
 * 端点策略：URDF/标定审阅场景默认“起点含、终点不含”，
 * 但两个端点都显式返回 AT_BOUNDARY 让页面展示明确状态。
 */
object CalibrationResolver {

    fun effective(
        version: CalibrationVersion,
        robotSerial: String,
        at: Instant?,
    ): CalibrationEffective? {
        if (version.robotSerial != robotSerial) return null
        val status = when {
            version.validFrom == null && version.validUntil == null -> CalibrationTimeStatus.UNBOUNDED
            at == null -> CalibrationTimeStatus.TIME_MISSING
            version.validFrom != null && at == version.validFrom -> CalibrationTimeStatus.AT_BOUNDARY
            version.validUntil != null && at == version.validUntil -> CalibrationTimeStatus.AT_BOUNDARY
            version.validFrom != null && at.isBefore(version.validFrom) -> CalibrationTimeStatus.NOT_YET_VALID
            version.validUntil != null && at.isAfter(version.validUntil) -> CalibrationTimeStatus.EXPIRED
            else -> CalibrationTimeStatus.ACTIVE
        }
        val applies = when (status) {
            CalibrationTimeStatus.ACTIVE, CalibrationTimeStatus.UNBOUNDED,
            CalibrationTimeStatus.AT_BOUNDARY,
            -> true
            else -> false
        }
        if (!applies) return null
        return CalibrationEffective(version.id, version.label, version.declarations, status)
    }

    fun statusOf(
        version: CalibrationVersion,
        robotSerial: String,
        at: Instant?,
    ): CalibrationTimeStatus {
        if (version.robotSerial != robotSerial) return CalibrationTimeStatus.SERIAL_MISMATCH
        return when {
            version.validFrom == null && version.validUntil == null -> CalibrationTimeStatus.UNBOUNDED
            at == null -> CalibrationTimeStatus.TIME_MISSING
            version.validFrom != null && at == version.validFrom -> CalibrationTimeStatus.AT_BOUNDARY
            version.validUntil != null && at == version.validUntil -> CalibrationTimeStatus.AT_BOUNDARY
            version.validFrom != null && at.isBefore(version.validFrom) -> CalibrationTimeStatus.NOT_YET_VALID
            version.validUntil != null && at.isAfter(version.validUntil) -> CalibrationTimeStatus.EXPIRED
            else -> CalibrationTimeStatus.ACTIVE
        }
    }
}
