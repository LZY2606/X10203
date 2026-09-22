package jcb.kin

import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * 4x4 齐次变换矩阵（列序存储 m[row * 4 + col]）。
 * URDF 约定：origin rpy 为绕定轴 x-y-z 的外旋（等价 Rz*Ry*Rx 内乘）。
 */
class Transform4 private constructor(val m: DoubleArray) {

    init {
        require(m.size == 16) { "变换矩阵必须是 4x4" }
    }

    operator fun get(row: Int, col: Int): Double = m[row * 4 + col]

    fun translate(): DoubleArray = doubleArrayOf(this[0, 3], this[1, 3], this[2, 3])

    fun rotation(): DoubleArray = DoubleArray(9) { idx ->
        val r = idx / 3
        val c = idx % 3
        this[r, c]
    }

    operator fun times(other: Transform4): Transform4 {
        val out = DoubleArray(16)
        for (row in 0 until 4) {
            for (col in 0 until 4) {
                var sum = 0.0
                for (k in 0 until 4) sum += this[row, k] * other[k, col]
                out[row * 4 + col] = sum
            }
        }
        return Transform4(out)
    }

    /** 刚体逆变换（R^T, -R^T * p）。 */
    fun inverse(): Transform4 {
        val out = DoubleArray(16)
        for (row in 0 until 3) {
            for (col in 0 until 3) {
                out[col * 4 + row] = this[row, col]
            }
        }
        for (col in 0 until 3) {
            var sum = 0.0
            for (k in 0 until 3) sum += out[col * 4 + k] * this[k, 3]
            out[col * 4 + 3] = -sum
        }
        out[15] = 1.0
        return Transform4(out)
    }

    fun isIdentity(eps: Double = 1e-12): Boolean {
        for (i in 0 until 16) {
            val expected = if (i % 5 == 0) 1.0 else 0.0
            if (kotlin.math.abs(m[i] - expected) > eps) return false
        }
        return true
    }

    /** 与另一个位姿的差异：平移最大差（米）+ 旋转角（弧度）。 */
    fun delta(other: Transform4): PoseDelta {
        val p1 = translate()
        val p2 = other.translate()
        var pd = 0.0
        for (i in 0 until 3) pd = maxOf(pd, kotlin.math.abs(p1[i] - p2[i]))
        var rDiff = 0.0
        if (pd.isFinite()) {
            val rel = this.inverse() * other
            val r = DoubleArray(9) { idx -> rel[idx / 3, idx % 3] }
            val cosAngle = ((r[0] + r[4] + r[8] - 1.0) / 2.0).coerceIn(-1.0, 1.0)
            rDiff = kotlin.math.acos(cosAngle)
        }
        return PoseDelta(pd, rDiff)
    }

    fun toRows(): List<List<Double>> = (0 until 4).map { row ->
        (0 until 4).map { col -> this[row, col] }
    }

    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (other !is Transform4) return false
        return m.contentEquals(other.m)
    }

    override fun hashCode(): Int = m.contentHashCode()

    companion object {
        fun identity(): Transform4 = Transform4(DoubleArray(16).also { it[0] = 1.0; it[5] = 1.0; it[10] = 1.0; it[15] = 1.0 })

        fun fromMatrix(rows: List<List<Double>>): Transform4 {
            require(rows.size == 4 && rows.all { it.size == 4 })
            return Transform4(DoubleArray(16) { idx -> rows[idx / 4][idx % 4] })
        }

        fun fromArray(m: DoubleArray): Transform4 {
            require(m.size == 16)
            return Transform4(m.copyOf())
        }

        /**
         * @param xyz 米
         * @param rpy 弧度，roll(x)-pitch(y)-yaw(z)
         */
        fun origin(xyz: DoubleArray, rpy: DoubleArray): Transform4 {
            require(xyz.size == 3 && rpy.size == 3)
            val cr = cos(rpy[0]); val sr = sin(rpy[0])
            val cp = cos(rpy[1]); val sp = sin(rpy[1])
            val cy = cos(rpy[2]); val sy = sin(rpy[2])
            // R = Rz(yaw) * Ry(pitch) * Rx(roll)
            val r = doubleArrayOf(
                cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
                sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
                -sp, cp * sr, cp * cr,
            )
            return Transform4(doubleArrayOf(
                r[0], r[1], r[2], xyz[0],
                r[3], r[4], r[5], xyz[1],
                r[6], r[7], r[8], xyz[2],
                0.0, 0.0, 0.0, 1.0,
            ))
        }

        fun translation(x: Double, y: Double, z: Double): Transform4 =
            origin(doubleArrayOf(x, y, z), DoubleArray(3))

        /** 绕指定轴旋转 angle（弧度），用于 revolute / continuous 关节。 */
        fun rotationAround(axis: DoubleArray, angle: Double): Transform4 {
            val norm = sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2])
            if (norm < 1e-12) return identity()
            val ux = axis[0] / norm
            val uy = axis[1] / norm
            val uz = axis[2] / norm
            val c = cos(angle)
            val s = sin(angle)
            val t = 1.0 - c
            return Transform4(doubleArrayOf(
                c + ux * ux * t, ux * uy * t - uz * s, ux * uz * t + uy * s, 0.0,
                uy * ux * t + uz * s, c + uy * uy * t, uy * uz * t - ux * s, 0.0,
                uz * ux * t - uy * s, uz * uy * t + ux * s, c + uz * uz * t, 0.0,
                0.0, 0.0, 0.0, 1.0,
            ))
        }

        /** 沿指定轴平移 distance（米），用于 prismatic 关节。 */
        fun translationAlong(axis: DoubleArray, distance: Double): Transform4 {
            val norm = sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2])
            if (norm < 1e-12) return identity()
            return Transform4(doubleArrayOf(
                1.0, 0.0, 0.0, axis[0] / norm * distance,
                0.0, 1.0, 0.0, axis[1] / norm * distance,
                0.0, 0.0, 1.0, axis[2] / norm * distance,
                0.0, 0.0, 0.0, 1.0,
            ))
        }
    }
}

data class PoseDelta(
    /** 平移分量最大偏差，单位米。 */
    val translationMeters: Double,
    /** 相对旋转角，单位弧度。 */
    val rotationRadians: Double,
) {
    fun below(tolerance: PathTolerance): Boolean =
        translationMeters <= tolerance.translationMeters &&
            rotationRadians <= tolerance.rotationRadians
}

data class PathTolerance(
    val translationMeters: Double = 1e-6,
    val rotationRadians: Double = 1e-6,
)
