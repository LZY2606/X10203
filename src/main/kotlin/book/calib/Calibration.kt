package book.calib

import book.math.Vec3
import kotlinx.serialization.Serializable
import java.time.Instant
import java.time.format.DateTimeParseException

/**
 * 标定变换与 URDF 分离、独立版本化。每条标定边可以：
 *  - 新增 URDF 中不存在的固定变换（new 边）
 *  - 覆盖同名 joint 的 origin（override 边，必须显式批准才会写入导出）
 */
@Serializable
data class CalibrationEdge(
    val id: String,
    /** override = 覆盖已有 joint origin；new = 新增固定 frame 边 */
    val kind: String,
    /** override 时为 joint 名；new 时使用 PARENT->CHILD 两端 */
    val jointName: String? = null,
    val parentFrame: String,
    val childFrame: String,
    val xyz: List<Double> = listOf(0.0, 0.0, 0.0),
    val rpyRad: List<Double> = listOf(0.0, 0.0, 0.0),
    val note: String = "",
)

@Serializable
data class CalibrationVersion(
    val id: String,
    val robotSerial: String,
    val version: Int,
    val note: String,
    val createdAt: String,
    val validFrom: String,
    val validUntil: String,
    val draft: Boolean,
    val approved: Boolean,
    val edges: List<CalibrationEdge>,
) {
    fun edgeById(edgeId: String) = edges.firstOrNull { it.id == edgeId }
}

enum class CalibValidityStatus { ACTIVE, FUTURE, EXPIRED, BOUNDARY_START, BOUNDARY_END, SERIAL_MISMATCH }

data class CalibValidity(val status: CalibValidityStatus, val reason: String) {
    val effective: Boolean get() = status == CalibValidityStatus.ACTIVE
}

object CalibRules {
    fun parseTime(raw: String): Instant? = try {
        Instant.parse(raw)
    } catch (_: DateTimeParseException) {
        null
    }

    /**
     * 生效规则：[validFrom, validUntil) 半开区间。
     * 边界明确报告：t == validFrom => BOUNDARY_START（视为生效，标注起点），
     * t == validUntil => BOUNDARY_END（已失效）。
     */
    fun evaluate(version: CalibrationVersion, serial: String, at: Instant): CalibValidity {
        if (version.robotSerial != serial) {
            return CalibValidity(CalibValidityStatus.SERIAL_MISMATCH,
                "标定面向序列号 ${version.robotSerial}，当前机器人为 $serial")
        }
        val from = parseTime(version.validFrom)
        val until = parseTime(version.validUntil)
        if (from == null || until == null) {
            return CalibValidity(CalibValidityStatus.EXPIRED, "标定时间字段无法解析")
        }
        return when {
            at == from -> CalibValidity(CalibValidityStatus.BOUNDARY_START, "恰好位于生效起点 $from，按生效处理")
            at == until -> CalibValidity(CalibValidityStatus.EXPIRED,
                "恰好位于生效终点 $until：半开区间 [from, until)，终点不生效")
            at.isBefore(from) -> CalibValidity(CalibValidityStatus.FUTURE,
                "尚未生效（$at < $from）")
            at.isAfter(until) || at == until -> CalibValidity(CalibValidityStatus.EXPIRED,
                "已过有效期（$at >= $until）")
            else -> CalibValidity(CalibValidityStatus.ACTIVE,
                "在有效期 [$from, $until) 内，序列号匹配")
        }
    }

    fun edgeVecs(e: CalibrationEdge): Pair<Vec3, Vec3> =
        Vec3(e.xyz[0], e.xyz.getOrElse(1) { 0.0 }, e.xyz.getOrElse(2) { 0.0 }) to
            Vec3(e.rpyRad[0], e.rpyRad.getOrElse(1) { 0.0 }, e.rpyRad.getOrElse(2) { 0.0 })
}
