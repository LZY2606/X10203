package atlas

import kotlin.math.*

/** 行主序 4x4 齐次变换矩阵 */
class Mat4(val m: DoubleArray) {
    init { require(m.size == 16) }

    companion object {
        fun identity() = Mat4(doubleArrayOf(
            1.0,0.0,0.0,0.0,
            0.0,1.0,0.0,0.0,
            0.0,0.0,1.0,0.0,
            0.0,0.0,0.0,1.0))

        fun translation(x: Double, y: Double, z: Double): Mat4 {
            val t = identity().m
            t[3] = x; t[7] = y; t[11] = z
            return Mat4(t)
        }

        fun rotX(a: Double): Mat4 {
            val c = cos(a); val s = sin(a)
            return Mat4(doubleArrayOf(1.0,0.0,0.0,0.0, 0.0,c,-s,0.0, 0.0,s,c,0.0, 0.0,0.0,0.0,1.0))
        }
        fun rotY(a: Double): Mat4 {
            val c = cos(a); val s = sin(a)
            return Mat4(doubleArrayOf(c,0.0,s,0.0, 0.0,1.0,0.0,0.0, -s,0.0,c,0.0, 0.0,0.0,0.0,1.0))
        }
        fun rotZ(a: Double): Mat4 {
            val c = cos(a); val s = sin(a)
            return Mat4(doubleArrayOf(c,-s,0.0,0.0, s,c,0.0,0.0, 0.0,0.0,1.0,0.0, 0.0,0.0,0.0,1.0))
        }

        /** URDF origin: 平移 + 固定轴 RPY (extrinsic XYZ) */
        fun fromXyzRpy(xyz: DoubleArray, rpy: DoubleArray): Mat4 =
            translation(xyz[0], xyz[1], xyz[2]) * rotZ(rpy[2]) * rotY(rpy[1]) * rotX(rpy[0])

        /** 绕任意轴旋转（罗德里格斯公式） */
        fun rotationAbout(axis: DoubleArray, angle: Double): Mat4 {
            val n = sqrt(axis[0]*axis[0] + axis[1]*axis[1] + axis[2]*axis[2])
            val x = axis[0]/n; val y = axis[1]/n; val z = axis[2]/n
            val c = cos(angle); val s = sin(angle); val v = 1 - c
            return Mat4(doubleArrayOf(
                x*x*v+c,     x*y*v-z*s,   x*z*v+y*s,   0.0,
                y*x*v+z*s,   y*y*v+c,     y*z*v-x*s,   0.0,
                z*x*v-y*s,   z*y*v+x*s,   z*z*v+c,     0.0,
                0.0, 0.0, 0.0, 1.0))
        }
    }

    operator fun times(o: Mat4): Mat4 {
        val r = DoubleArray(16)
        for (i in 0..3) for (j in 0..3) {
            var s = 0.0
            for (k in 0..3) s += m[i*4+k] * o.m[k*4+j]
            r[i*4+j] = s
        }
        return Mat4(r)
    }

    /** 刚体变换求逆 */
    fun inverseRigid(): Mat4 {
        val r = DoubleArray(16)
        for (i in 0..2) for (j in 0..2) r[i*4+j] = m[j*4+i]
        val tx = m[3]; val ty = m[7]; val tz = m[11]
        r[3]  = -(r[0]*tx + r[1]*ty + r[2]*tz)
        r[7]  = -(r[4]*tx + r[5]*ty + r[6]*tz)
        r[11] = -(r[8]*tx + r[9]*ty + r[10]*tz)
        r[15] = 1.0
        return Mat4(r)
    }

    fun translationErrorVs(o: Mat4): Double {
        val dx = m[3]-o.m[3]; val dy = m[7]-o.m[7]; val dz = m[11]-o.m[11]
        return sqrt(dx*dx + dy*dy + dz*dz)
    }

    /** 与另一矩阵的旋转差（角度，度） */
    fun rotationErrorDegVs(o: Mat4): Double {
        // trace(R1^T R2)
        var tr = 0.0
        for (i in 0..2) for (j in 0..2) tr += m[j*4+i] * o.m[j*4+i]
        val cosA = ((tr - 1.0) / 2.0).coerceIn(-1.0, 1.0)
        return Math.toDegrees(acos(cosA))
    }

    fun maxElementDiff(o: Mat4): Double {
        var d = 0.0
        for (i in 0..15) d = max(d, abs(m[i] - o.m[i]))
        return d
    }

    fun toList(): List<Double> = m.toList()
}
