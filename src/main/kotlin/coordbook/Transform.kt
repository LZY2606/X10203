package coordbook

import kotlin.math.*

/** 4x4 homogeneous transform, row-major. */
data class Transform(val m: DoubleArray) {
    init { require(m.size == 16) }

    companion object {
        val IDENTITY = Transform(DoubleArray(16) { i -> if (i % 5 == 0) 1.0 else 0.0 })

        fun of(r: Array<DoubleArray>, t: DoubleArray): Transform {
            val m = DoubleArray(16)
            for (i in 0..2) for (j in 0..2) m[i * 4 + j] = r[i][j]
            m[3] = t[0]; m[7] = t[1]; m[11] = t[2]; m[15] = 1.0
            return Transform(m)
        }

        /** URDF origin: xyz + roll/pitch/yaw (radians, fixed-axis RPY = Rz*Ry*Rx). */
        fun fromXyzRpy(xyz: DoubleArray, rpy: DoubleArray): Transform {
            val (r, p, y) = rpy
            val cr = cos(r); val sr = sin(r); val cp = cos(p); val sp = sin(p); val cy = cos(y); val sy = sin(y)
            val rot = arrayOf(
                doubleArrayOf(cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
                doubleArrayOf(sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
                doubleArrayOf(-sp, cp * sr, cp * cr),
            )
            return of(rot, xyz)
        }

        fun rotationAboutAxis(axis: DoubleArray, angle: Double): Transform {
            val n = sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2])
            if (n < 1e-12) return IDENTITY
            val x = axis[0] / n; val y = axis[1] / n; val z = axis[2] / n
            val c = cos(angle); val s = sin(angle); val v = 1 - c
            val rot = arrayOf(
                doubleArrayOf(x * x * v + c, x * y * v - z * s, x * z * v + y * s),
                doubleArrayOf(y * x * v + z * s, y * y * v + c, y * z * v - x * s),
                doubleArrayOf(z * x * v - y * s, z * y * v + x * s, z * z * v + c),
            )
            return of(rot, doubleArrayOf(0.0, 0.0, 0.0))
        }

        fun translation(d: DoubleArray) = of(
            arrayOf(doubleArrayOf(1.0, 0.0, 0.0), doubleArrayOf(0.0, 1.0, 0.0), doubleArrayOf(0.0, 0.0, 1.0)), d)
    }

    fun compose(o: Transform): Transform {
        val r = DoubleArray(16)
        for (i in 0..3) for (j in 0..3) {
            var s = 0.0
            for (k in 0..3) s += m[i * 4 + k] * o.m[k * 4 + j]
            r[i * 4 + j] = s
        }
        return Transform(r)
    }

    fun inverse(): Transform {
        // Rigid transform inverse: R^T, -R^T t
        val r = DoubleArray(16)
        for (i in 0..2) for (j in 0..2) r[i * 4 + j] = m[j * 4 + i]
        val t = doubleArrayOf(m[3], m[7], m[11])
        for (i in 0..2) {
            r[i * 4 + 3] = -(r[i * 4] * t[0] + r[i * 4 + 1] * t[1] + r[i * 4 + 2] * t[2])
        }
        r[15] = 1.0
        return Transform(r)
    }

    fun translationNorm() = sqrt(m[3] * m[3] + m[7] * m[7] + m[11] * m[11])

    /** Rotation angle of the rotational part, radians in [0, pi]. */
    fun rotationAngle(): Double {
        val tr = m[0] + m[5] + m[10]
        val c = ((tr - 1.0) / 2.0).coerceIn(-1.0, 1.0)
        return acos(c)
    }

    /** |a^-1 * b| residual: translation norm + rotation angle. */
    fun residualTo(o: Transform): Pair<Double, Double> {
        val d = this.inverse().compose(o)
        return d.translationNorm() to d.rotationAngle()
    }

    fun rows(): List<List<Double>> = (0..3).map { i -> (0..3).map { j -> m[i * 4 + j] } }

    fun approxEquals(o: Transform, eps: Double = 1e-9): Boolean =
        (0 until 16).all { abs(m[it] - o.m[it]) < eps }
}
