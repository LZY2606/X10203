package atlas

import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

/** Row-major 4x4 homogeneous transform. */
class Mat4(val m: DoubleArray) {
    init {
        require(m.size == 16) { "Mat4 needs 16 elements" }
    }

    companion object {
        fun identity() = Mat4(DoubleArray(16) { if (it % 5 == 0) 1.0 else 0.0 })

        fun of(rows: Array<DoubleArray>) = Mat4(DoubleArray(16) { rows[it / 4][it % 4] })

        fun translation(x: Double, y: Double, z: Double): Mat4 {
            val r = identity()
            r.m[3] = x; r.m[7] = y; r.m[11] = z
            return r
        }

        /** URDF fixed-axis roll-pitch-yaw: R = Rz(yaw) * Ry(pitch) * Rx(roll), radians. */
        fun fromXyzRpy(xyz: DoubleArray, rpy: DoubleArray): Mat4 {
            val roll = rpy[0]; val pitch = rpy[1]; val yaw = rpy[2]
            val cr = cos(roll); val sr = sin(roll)
            val cp = cos(pitch); val sp = sin(pitch)
            val cy = cos(yaw); val sy = sin(yaw)
            val r = identity()
            r.m[0] = cy * cp; r.m[1] = cy * sp * sr - sy * cr; r.m[2] = cy * sp * cr + sy * sr
            r.m[4] = sy * cp; r.m[5] = sy * sp * sr + cy * cr; r.m[6] = sy * sp * cr - cy * sr
            r.m[8] = -sp; r.m[9] = cp * sr; r.m[10] = cp * cr
            r.m[3] = xyz[0]; r.m[7] = xyz[1]; r.m[11] = xyz[2]
            return r
        }

        fun axisAngle(axis: DoubleArray, angle: Double): Mat4 {
            val raw = sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2])
            val len = if (raw == 0.0) 1.0 else raw
            val x = axis[0] / len; val y = axis[1] / len; val z = axis[2] / len
            val c = cos(angle); val s = sin(angle); val v = 1 - c
            return of(
                arrayOf(
                    doubleArrayOf(x * x * v + c, x * y * v - z * s, x * z * v + y * s, 0.0),
                    doubleArrayOf(y * x * v + z * s, y * y * v + c, y * z * v - x * s, 0.0),
                    doubleArrayOf(z * x * v - y * s, z * y * v + x * s, z * z * v + c, 0.0),
                    doubleArrayOf(0.0, 0.0, 0.0, 1.0)
                )
            )
        }
    }

    operator fun times(o: Mat4): Mat4 {
        val r = DoubleArray(16)
        for (i in 0..3) for (j in 0..3) {
            var s = 0.0
            for (k in 0..3) s += m[i * 4 + k] * o.m[k * 4 + j]
            r[i * 4 + j] = s
        }
        return Mat4(r)
    }

    fun inverseRigid(): Mat4 {
        val r = identity()
        for (i in 0..2) for (j in 0..2) r.m[i * 4 + j] = m[j * 4 + i]
        for (i in 0..2) {
            var s = 0.0
            for (k in 0..2) s += r.m[i * 4 + k] * m[k * 4 + 3]
            r.m[i * 4 + 3] = -s
        }
        return r
    }

    fun maxAbsDiff(o: Mat4): Double {
        var d = 0.0
        for (i in 0..15) d = maxOf(d, abs(m[i] - o.m[i]))
        return d
    }

    fun rows(): List<List<Double>> = (0..3).map { i -> (0..3).map { j -> m[i * 4 + j] } }
}
