package jcb.kin

import kotlin.math.*

/** 齐次 4x4 变换（行主序存储）。表示 T_parent_child：在 parent 坐标系下表达 child。 */
data class Transform(val m: DoubleArray = DoubleArray(16)) {
    init { require(m.size == 16) }

    operator fun get(row: Int, col: Int): Double = m[row * 4 + col]

    override fun equals(other: Any?): Boolean =
        other is Transform && m.contentEquals(other.m)
    override fun hashCode(): Int = m.contentHashCode()

    fun copy(): Transform = Transform(m.copyOf())

    fun toMatrixRows(): List<List<Double>> = (0..3).map { r -> (0..3).map { c -> this[r, c] } }

    companion object {
        fun identity(): Transform = Transform().apply {
            for (i in 0..3) m[i * 4 + i] = 1.0
        }

        fun translation(x: Double, y: Double, z: Double): Transform = identity().apply {
            m[3] = x; m[7] = y; m[11] = z
        }

        /** URDF 固定轴 RPY：先绕 x(roll)、再 y(pitch)、最后 z(yaw)，即 R = Rz * Ry * Rx。角度单位：弧度。 */
        fun rpy(roll: Double, pitch: Double, yaw: Double): Transform {
            val cr = cos(roll); val sr = sin(roll)
            val cp = cos(pitch); val sp = sin(pitch)
            val cy = cos(yaw); val sy = sin(yaw)
            val r = DoubleArray(16)
            r[0] = cy * cp
            r[1] = cy * sp * sr - sy * cr
            r[2] = cy * sp * cr + sy * sr
            r[4] = sy * cp
            r[5] = sy * sp * sr + cy * cr
            r[6] = sy * sp * cr - cy * sr
            r[8] = -sp
            r[9] = cp * sr
            r[10] = cp * cr
            r[15] = 1.0
            return Transform(r)
        }

        /** 四元数 (x, y, z, w)，URDF calibration 扩展使用。 */
        fun quaternion(x: Double, y: Double, z: Double, w: Double): Transform {
            val n = sqrt(x * x + y * y + z * z + w * w)
            val qx = x / n; val qy = y / n; val qz = z / n; val qw = w / n
            val r = DoubleArray(16)
            r[0] = 1 - 2 * (qy * qy + qz * qz)
            r[1] = 2 * (qx * qy - qz * qw)
            r[2] = 2 * (qx * qz + qy * qw)
            r[4] = 2 * (qx * qy + qz * qw)
            r[5] = 1 - 2 * (qx * qx + qz * qz)
            r[6] = 2 * (qy * qz - qx * qw)
            r[8] = 2 * (qx * qz - qy * qw)
            r[9] = 2 * (qy * qz + qx * qw)
            r[10] = 1 - 2 * (qx * qx + qy * qy)
            r[15] = 1.0
            return Transform(r)
        }

        fun xyzRpy(xyz: DoubleArray, rpy: DoubleArray): Transform {
            require(xyz.size == 3 && rpy.size == 3)
            return rpy(rpy[0], rpy[1], rpy[2]).also { t ->
                t.m[3] = xyz[0]; t.m[7] = xyz[1]; t.m[11] = xyz[2]
            }
        }
    }
}

operator fun Transform.times(other: Transform): Transform {
    val a = this.m; val b = other.m; val out = DoubleArray(16)
    for (r in 0..3) for (c in 0..3) {
        var s = 0.0
        for (k in 0..3) s += a[r * 4 + k] * b[k * 4 + c]
        out[r * 4 + c] = s
    }
    return Transform(out)
}

fun Transform.inverse(): Transform {
    val a = m
    val out = DoubleArray(16)
    // R^T
    for (r in 0..2) for (c in 0..2) out[r * 4 + c] = a[c * 4 + r]
    // t' = -R^T * t
    for (r in 0..2) {
        var s = 0.0
        for (k in 0..2) s -= out[r * 4 + k] * a[k * 4 + 3]
        out[r * 4 + 3] = s
    }
    out[15] = 1.0
    return Transform(out)
}

/** 平移距离（米）与旋转角度（弧度）误差，用于多路径一致性比较。 */
data class PoseDiff(val translationMeters: Double, val rotationRadians: Double) {
    companion object {
        fun between(a: Transform, b: Transform): PoseDiff {
            val dx = a[0, 3] - b[0, 3]
            val dy = a[1, 3] - b[1, 3]
            val dz = a[2, 3] - b[2, 3]
            val dist = sqrt(dx * dx + dy * dy + dz * dz)
            var trace = 0.0
            for (i in 0..2) trace += a[i, i] - b[i, i]
            // 相对旋转 R_a^-1 R_b
            var t = 0.0
            for (i in 0..2) for (j in 0..2) t += a[j, i] * b[j, i]
            val cosAngle = ((t - 1.0) / 2.0).coerceIn(-1.0, 1.0)
            return PoseDiff(dist, acos(cosAngle))
        }
    }
}
