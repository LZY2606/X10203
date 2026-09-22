package jcb.model

import java.time.Instant

/** 标定包：独立版本化，仅对指定机器人序列号与时间段生效。 */
data class CalibrationVersion(
    val id: String,
    val robotSerial: String,
    val label: String,
    val validFrom: Instant?,
    val validUntil: Instant?,
    val createdAt: Instant,
    val note: String,
    val declarations: List<CalibrationDeclaration>,
)

/** 单条标定声明：parent -> child 的额外/覆盖变换。 */
data class CalibrationDeclaration(
    val id: String,
    val parentFrame: String,
    val childFrame: String,
    val xyzMeters: DoubleArray,
    val rpyRadians: DoubleArray,
    val comment: String,
) {
    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (other !is CalibrationDeclaration) return false
        return id == other.id
    }

    override fun hashCode(): Int = id.hashCode()
}

/** 现场关节快照：冻结后可对照不同标定版本。 */
data class JointSnapshot(
    val id: String,
    val robotSerial: String,
    val frozenAt: Instant,
    /** joint 名称 -> 关节值（revolute/continuous 为弧度，prismatic 为米；fixed 不出现）。 */
    val values: Map<String, Double>,
    val note: String,
)
