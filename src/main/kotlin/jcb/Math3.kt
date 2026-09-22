package jcb

import kotlin.math.*

/** 4x4 homogeneous transform, row-major. */
data class Mat4(val r: Array<DoubleArray>) {
    companion object {
        fun identity(): Mat4 = Mat4(Array(4) { i -> DoubleArray(4) { j -> if (i == j) 1.0 else 0.0 } })

        fun translation(x: Double, y: Double, z: Double): Mat4 {
            val m = identity()
            m.r[0][3] = x; m.r[1][3] = y; m.r[2][3] = z
            return m
        }

        /** URDF rpy: fixed-axis roll-pitch-yaw, R = Rz(yaw)*Ry(pitch)*Rx(roll). */
        fun fromXyzRpy(xyz: DoubleArray, rpy: DoubleArray): Mat4 {
            val (roll, pitch, yaw) = rpy
            val cr = cos(roll); val sr = sin(roll)
            val cp = cos(pitch); val sp = sin(pitch)
            val cy = cos(yaw); val sy = sin(yaw)
            val m = identity()
            m.r[0][0] = cy * cp; m.r[0][1] = cy * sp * sr - sy * cr; m.r[0][2] = cy * sp * cr + sy * sr
            m.r[1][0] = sy * cp; m.r[1][1] = sy * sp * sr + cy * cr; m.r[1][2] = sy * sp * cr - cy * sr
            m.r[2][0] = -sp;     m.r[2][1] = cp * sr;               m.r[2][2] = cp * cr
            m.r[0][3] = xyz[0]; m.r[1][3] = xyz[1]; m.r[2][3] = xyz[2]
            return m
        }

        fun axisAngle(axis: DoubleArray, angle: Double): Mat4 {
            val n = norm(axis)
            val (x, y, z) = n
            val c = cos(angle); val s = sin(angle); val v = 1 - c
            val m = identity()
            m.r[0][0] = x * x * v + c;     m.r[0][1] = x * y * v - z * s; m.r[0][2] = x * z * v + y * s
            m.r[1][0] = y * x * v + z * s; m.r[1][1] = y * y * v + c;     m.r[1][2] = y * z * v - x * s
            m.r[2][0] = z * x * v - y * s; m.r[2][1] = z * y * v + x * s; m.r[2][2] = z * z * v + c
            return m
        }

        fun norm(v: DoubleArray): DoubleArray {
            val l = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
            return if (l == 0.0) doubleArrayOf(0.0, 0.0, 1.0) else doubleArrayOf(v[0] / l, v[1] / l, v[2] / l)
        }
    }

    fun times(o: Mat4): Mat4 {
        val out = Array(4) { DoubleArray(4) }
        for (i in 0..3) for (j in 0..3) {
            var s = 0.0
            for (k in 0..3) s += r[i][k] * o.r[k][j]
            out[i][j] = s
        }
        return Mat4(out)
    }

    fun inverse(): Mat4 {
        val out = identity()
        for (i in 0..2) for (j in 0..2) out.r[i][j] = r[j][i]
        for (i in 0..2) {
            out.r[i][3] = -(out.r[i][0] * r[0][3] + out.r[i][1] * r[1][3] + out.r[i][2] * r[2][3])
        }
        return out
    }

    /** Position + rotation-angle deviation between two transforms. */
    fun deviation(o: Mat4): Double {
        val d = this.inverse().times(o)
        val dx = d.r[0][3]; val dy = d.r[1][3]; val dz = d.r[2][3]
        val trans = sqrt(dx * dx + dy * dy + dz * dz)
        val trace = d.r[0][0] + d.r[1][1] + d.r[2][2]
        val cosA = ((trace - 1.0) / 2.0).coerceIn(-1.0, 1.0)
        val rot = abs(acos(cosA))
        return trans + rot
    }

    fun toRows(): List<List<Double>> = r.map { it.toList() }

    override fun equals(other: Any?): Boolean =
        other is Mat4 && r.indices.all { i -> r[i].indices.all { j -> abs(r[i][j] - other.r[i][j]) < 1e-9 } }
    override fun hashCode(): Int = r.sumOf { it.sum().toInt() }
}
