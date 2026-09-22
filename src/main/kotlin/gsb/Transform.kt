package gsb

import kotlin.math.acos
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

/** Rigid 4x4 homogeneous transform. Angles are always radians internally. */
class Transform(val m: Array<DoubleArray>) {
    init {
        require(m.size == 4 && m.all { it.size == 4 }) { "transform must be 4x4" }
    }

    operator fun times(other: Transform): Transform {
        val r = Array(4) { DoubleArray(4) }
        for (i in 0..3) for (j in 0..3) {
            var s = 0.0
            for (k in 0..3) s += m[i][k] * other.m[k][j]
            r[i][j] = s
        }
        return Transform(r)
    }

    /** Inverse of a rigid transform: R^T and -R^T t. */
    fun inverse(): Transform {
        val r = Array(4) { DoubleArray(4) }
        for (i in 0..2) for (j in 0..2) r[i][j] = m[j][i]
        for (i in 0..2) {
            var s = 0.0
            for (k in 0..2) s -= r[i][k] * m[k][3]
            r[i][3] = s
        }
        r[3][3] = 1.0
        return Transform(r)
    }

    fun position(): Triple<Double, Double, Double> = Triple(m[0][3], m[1][3], m[2][3])

    /** Rotation angle (rad) of the rotation part relative to identity. */
    fun rotationErrorRad(): Double {
        val trace = m[0][0] + m[1][1] + m[2][2]
        val c = ((trace - 1.0) / 2.0).coerceIn(-1.0, 1.0)
        return acos(c)
    }

    fun translationNorm(): Double {
        val (x, y, z) = position()
        return sqrt(x * x + y * y + z * z)
    }

    fun errorNorm(): Double = translationNorm() + rotationErrorRad()

    fun isIdentity(eps: Double = 1e-9): Boolean = errorNorm() < eps

    /** Row-major flattening, 16 numbers. */
    fun toFlatList(): List<Double> = m.flatMap { it.toList() }

    companion object {
        fun identity(): Transform = Transform(Array(4) { i -> DoubleArray(4) { j -> if (i == j) 1.0 else 0.0 } })

        fun translation(x: Double, y: Double, z: Double): Transform {
            val t = identity()
            t.m[0][3] = x; t.m[1][3] = y; t.m[2][3] = z
            return t
        }

        /** URDF origin: fixed-axis roll-pitch-yaw (radians) plus xyz translation. */
        fun fromXyzRpy(x: Double, y: Double, z: Double, roll: Double, pitch: Double, yaw: Double): Transform {
            val cr = cos(roll); val sr = sin(roll)
            val cp = cos(pitch); val sp = sin(pitch)
            val cy = cos(yaw); val sy = sin(yaw)
            val r = Array(4) { DoubleArray(4) }
            r[0][0] = cy * cp; r[0][1] = cy * sp * sr - sy * cr; r[0][2] = cy * sp * cr + sy * sr
            r[1][0] = sy * cp; r[1][1] = sy * sp * sr + cy * cr; r[1][2] = sy * sp * cr - cy * sr
            r[2][0] = -sp; r[2][1] = cp * sr; r[2][2] = cp * cr
            r[0][3] = x; r[1][3] = y; r[2][3] = z; r[3][3] = 1.0
            return Transform(r)
        }

        fun rotationAxisAngle(ax: Double, ay: Double, az: Double, angle: Double): Transform {
            val len = sqrt(ax * ax + ay * ay + az * az)
            if (len < 1e-12) return identity()
            val x = ax / len; val y = ay / len; val z = az / len
            val c = cos(angle); val s = sin(angle); val v = 1 - c
            val r = Array(4) { DoubleArray(4) }
            r[0][0] = x * x * v + c; r[0][1] = x * y * v - z * s; r[0][2] = x * z * v + y * s
            r[1][0] = y * x * v + z * s; r[1][1] = y * y * v + c; r[1][2] = y * z * v - x * s
            r[2][0] = z * x * v - y * s; r[2][1] = z * y * v + x * s; r[2][2] = z * z * v + c
            r[3][3] = 1.0
            return Transform(r)
        }
    }
}
