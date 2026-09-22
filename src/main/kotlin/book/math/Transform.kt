package book.math

import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

/** 3 维向量（平移，单位：米；角度一律弧度内部处理）。 */
data class Vec3(val x: Double, val y: Double, val z: Double) {
    operator fun plus(o: Vec3) = Vec3(x + o.x, y + o.y, z + o.z)
    operator fun minus(o: Vec3) = Vec3(x - o.x, y - o.y, z - o.z)
    operator fun times(s: Double) = Vec3(x * s, y * s, z * s)
    fun norm(): Double = sqrt(x * x + y * y + z * z)

    companion object {
        val ZERO = Vec3(0.0, 0.0, 0.0)
    }
}

/**
 * 4x4 齐次变换（行主序 m[row][col]），表示 T = [R p; 0 1]。
 * 坐标约定遵循 URDF：child = T * parent 方向上的关节变换。
 */
data class Mat4(val m: Array<DoubleArray>) {

    init {
        require(m.size == 4 && m.all { it.size == 4 }) { "Mat4 必须为 4x4" }
    }

    operator fun times(o: Mat4): Mat4 {
        val r = Array(4) { DoubleArray(4) }
        for (i in 0 until 4) {
            for (j in 0 until 4) {
                var s = 0.0
                for (k in 0 until 4) s += m[i][k] * o.m[k][j]
                r[i][j] = s
            }
        }
        return Mat4(r)
    }

    fun translation(): Vec3 = Vec3(m[0][3], m[1][3], m[2][3])

    /** 把该矩阵作用于一个点。 */
    fun applyPoint(p: Vec3): Vec3 = Vec3(
        m[0][0] * p.x + m[0][1] * p.y + m[0][2] * p.z + m[0][3],
        m[1][0] * p.x + m[1][1] * p.y + m[1][2] * p.z + m[1][3],
        m[2][0] * p.x + m[2][1] * p.y + m[2][2] * p.z + m[2][3],
    )

    fun inverse(): Mat4 {
        val r = Array(4) { DoubleArray(4) }
        for (i in 0 until 3) for (j in 0 until 3) r[j][i] = m[i][j]
        for (i in 0 until 3) {
            r[i][3] = -(r[i][0] * m[0][3] + r[i][1] * m[1][3] + r[i][2] * m[2][3])
        }
        r[3][3] = 1.0
        return Mat4(r)
    }

    /** 平移误差（米）+ 旋转误差（弧度，由 R 的迹换算），用于比较多条候选路径。 */
    fun diffFrom(o: Mat4): TransformDiff {
        val dT = this * o.inverse()
        val dt = dT.translation().norm()
        val tr = dT.m[0][0] + dT.m[1][1] + dT.m[2][2]
        val c = ((tr - 1.0) / 2.0).coerceIn(-1.0, 1.0)
        val dAngle = kotlin.math.acos(c)
        return TransformDiff(dt, dAngle)
    }

    fun rows(): List<List<Double>> = m.map { it.toList() }

    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (other !is Mat4) return false
        return m.contentDeepEquals(other.m)
    }

    override fun hashCode(): Int = m.contentDeepHashCode()

    companion object {
        fun identity(): Mat4 = Mat4(Array(4) { r ->
            DoubleArray(4) { c -> if (r == c) 1.0 else 0.0 }
        })

        fun translation(p: Vec3): Mat4 = identity().also {
            it.m[0][3] = p.x; it.m[1][3] = p.y; it.m[2][3] = p.z
        }

        /** URDF rpy：绕固定轴 XYZ 依次旋转（extrinsic XYZ = Rz*Ry*Rx）。 */
        fun rpy(roll: Double, pitch: Double, yaw: Double): Mat4 {
            val cr = cos(roll); val sr = sin(roll)
            val cp = cos(pitch); val sp = sin(pitch)
            val cy = cos(yaw); val sy = sin(yaw)
            val rx = Array(4) { DoubleArray(4) { if (it / 4 == it % 4) 1.0 else 0.0 } }
            rx[1][1] = cr; rx[1][2] = -sr; rx[2][1] = sr; rx[2][2] = cr
            val ry = Array(4) { DoubleArray(4) { if (it / 4 == it % 4) 1.0 else 0.0 } }
            ry[0][0] = cp; ry[0][2] = sp; ry[2][0] = -sp; ry[2][2] = cp
            val rz = Array(4) { DoubleArray(4) { if (it / 4 == it % 4) 1.0 else 0.0 } }
            rz[0][0] = cy; rz[0][1] = -sy; rz[1][0] = sy; rz[1][1] = cy
            return Mat4(rz) * Mat4(ry) * Mat4(rx)
        }

        fun fromXyzRpy(xyz: Vec3, rpy: Vec3): Mat4 {
            val t = rpy(rpy.x, rpy.y, rpy.z)
            t.m[0][3] = xyz.x; t.m[1][3] = xyz.y; t.m[2][3] = xyz.z
            return t
        }
    }
}

data class TransformDiff(val translationMeters: Double, val rotationRadians: Double) {
    fun rotationDegrees(): Double = Math.toDegrees(rotationRadians)
}
