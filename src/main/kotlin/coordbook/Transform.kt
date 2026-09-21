package coordbook

import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * 4x4 齐次变换，行主序 double[16]：
 *   m[row*4 + col]，T 将子 frame 的点表达为父 frame 坐标：p_parent = T * p_child。
 * 平移单位米；旋转由 URDF roll/pitch/yaw（弧度，固定轴 XYZ 外旋）或轴角生成。
 */
data class Transform(val m: DoubleArray = DoubleArray(16).also {
        it[0] = 1.0; it[5] = 1.0; it[10] = 1.0; it[15] = 1.0
    }) {

    init {
        require(m.size == 16)
    }

    fun copyT(): Transform = Transform(m.copyOf())

    operator fun times(other: Transform): Transform {
        val r = DoubleArray(16)
        for (row in 0 until 4) {
            for (col in 0 until 4) {
                var sum = 0.0
                for (k in 0 until 4) sum += m[row * 4 + k] * other.m[k * 4 + col]
                r[row * 4 + col] = sum
            }
        }
        return Transform(r)
    }

    fun inverse(): Transform {
        // 分块：R^T 与 -R^T*t
        val r = DoubleArray(16)
        for (i in 0 until 3) {
            for (j in 0 until 3) r[j * 4 + i] = m[i * 4 + j]
        }
        for (j in 0 until 3) {
            var s = 0.0
            for (i in 0 until 3) s += r[j * 4 + i] * m[i * 4 + 3]
            r[j * 4 + 3] = -s
        }
        r[15] = 1.0
        return Transform(r)
    }

    val translation: DoubleArray get() = doubleArrayOf(m[3], m[7], m[11])

    /** 旋转部分到单位四元数 (x,y,z,w)。 */
    fun quaternion(): DoubleArray {
        val t = m[0] + m[5] + m[10]
        val q = DoubleArray(4)
        when {
            t > 0 -> {
                val s = sqrt(t + 1.0) * 2.0
                q[3] = 0.25 * s
                q[0] = (m[6] - m[9]) / s
                q[1] = (m[8] - m[2]) / s
                q[2] = (m[1] - m[4]) / s
            }
            m[0] > m[5] && m[0] > m[10] -> {
                val s = sqrt(1.0 + m[0] - m[5] - m[10]) * 2.0
                q[3] = (m[6] - m[9]) / s
                q[0] = 0.25 * s
                q[1] = (m[4] + m[1]) / s
                q[2] = (m[8] + m[2]) / s
            }
            m[5] > m[10] -> {
                val s = sqrt(1.0 + m[5] - m[0] - m[10]) * 2.0
                q[3] = (m[8] - m[2]) / s
                q[0] = (m[4] + m[1]) / s
                q[1] = 0.25 * s
                q[2] = (m[9] + m[6]) / s
            }
            else -> {
                val s = sqrt(1.0 + m[10] - m[0] - m[5]) * 2.0
                q[3] = (m[1] - m[4]) / s
                q[0] = (m[8] + m[2]) / s
                q[1] = (m[9] + m[6]) / s
                q[2] = 0.25 * s
            }
        }
        val n = sqrt(q.sumOf { it * it })
        return DoubleArray(4) { q[it] / n }
    }

    fun matrixRows(): List<List<Double>> = (0 until 4).map { row ->
        (0 until 4).map { col -> m[row * 4 + col] }
    }

    /** 平移误差（米）+ 姿态测地角误差（弧度），用于一致性判定与展示。 */
    fun diff(other: Transform): TransformDelta {
        val t1 = translation
        val t2 = other.translation
        val transErr = hypot(
            hypot(t1[0] - t2[0], t1[1] - t2[1]),
            t1[2] - t2[2],
        )
        val q1 = quaternion()
        val q2 = other.quaternion()
        var dot = 0.0
        for (i in 0 until 4) dot += q1[i] * q2[i]
        dot = abs(dot).coerceAtMost(1.0)
        return TransformDelta(transErr, 2.0 * kotlin.math.acos(dot))
    }

    companion object {
        fun identity(): Transform = Transform()

        fun translation(x: Double, y: Double, z: Double): Transform {
            val t = Transform()
            t.m[3] = x; t.m[7] = y; t.m[11] = z
            return t
        }

        /** URDF 约定：绕固定（外旋）轴 X、Y、Z 依次旋转，角度单位弧度。 */
        fun fromRpy(x: Double, y: Double, z: Double): Transform =
            rotZ(z) * rotY(y) * rotX(x)

        fun fromAxisAngle(axis: DoubleArray, angle: Double): Transform {
            val n = sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2])
            if (n == 0.0) return identity()
            val ux = axis[0] / n; val uy = axis[1] / n; val uz = axis[2] / n
            val c = cos(angle); val s = sin(angle); val k = 1.0 - c
            val r = DoubleArray(16)
            r[0] = c + ux * ux * k
            r[1] = ux * uy * k - uz * s
            r[2] = ux * uz * k + uy * s
            r[4] = uy * ux * k + uz * s
            r[5] = c + uy * uy * k
            r[6] = uy * uz * k - ux * s
            r[8] = uz * ux * k - uy * s
            r[9] = uz * uy * k + ux * s
            r[10] = c + uz * uz * k
            r[15] = 1.0
            return Transform(r)
        }

        fun rotX(a: Double): Transform {
            val r = Transform()
            r.m[5] = cos(a); r.m[6] = -sin(a)
            r.m[9] = sin(a); r.m[10] = cos(a)
            return r
        }

        fun rotY(a: Double): Transform {
            val r = Transform()
            r.m[0] = cos(a); r.m[2] = sin(a)
            r.m[8] = -sin(a); r.m[10] = cos(a)
            return r
        }

        fun rotZ(a: Double): Transform {
            val r = Transform()
            r.m[0] = cos(a); r.m[1] = -sin(a)
            r.m[4] = sin(a); r.m[5] = cos(a)
            return r
        }

        /** 四元数 (x,y,z,w) 平均 + 平移算术平均（弦 L2 均值），候选多于一个时使用。 */
        fun average(transforms: List<Transform>): Transform {
            require(transforms.isNotEmpty())
            if (transforms.size == 1) return transforms[0]
            val qm = DoubleArray(4)
            var signBase = transforms[0].quaternion()
            for (t in transforms) {
                val q = t.quaternion()
                var dot = 0.0
                for (i in 0 until 4) dot += q[i] * signBase[i]
                val f = if (dot < 0) -1.0 else 1.0
                for (i in 0 until 4) qm[i] += f * q[i]
            }
            val n = sqrt(qm.sumOf { it * it })
            val x = qm[0] / n; val y = qm[1] / n; val z = qm[2] / n; val w = qm[3] / n
            val tx = transforms.sumOf { it.m[3] } / transforms.size
            val ty = transforms.sumOf { it.m[7] } / transforms.size
            val tz = transforms.sumOf { it.m[11] } / transforms.size
            return fromQuaternion(doubleArrayOf(x, y, z, w), doubleArrayOf(tx, ty, tz))
        }

        fun fromQuaternion(q: DoubleArray, t: DoubleArray): Transform {
            val x = q[0]; val y = q[1]; val z = q[2]; val w = q[3]
            val r = DoubleArray(16)
            r[0] = 1 - 2 * (y * y + z * z)
            r[1] = 2 * (x * y - z * w)
            r[2] = 2 * (x * z + y * w)
            r[4] = 2 * (x * y + z * w)
            r[5] = 1 - 2 * (x * x + z * z)
            r[6] = 2 * (y * z - x * w)
            r[8] = 2 * (x * z - y * w)
            r[9] = 2 * (y * z + x * w)
            r[10] = 1 - 2 * (x * x + y * y)
            r[3] = t[0]; r[7] = t[1]; r[11] = t[2]; r[15] = 1.0
            return Transform(r)
        }
    }
}

data class TransformDelta(val translationMeters: Double, val rotationRadians: Double)

/** 解析 URDF 的 "x y z" / "r p y" 三元串，缺省返回零。 */
fun parseTriplet(text: String?): DoubleArray {
    if (text.isNullOrBlank()) return DoubleArray(3)
    val parts = text.trim().split(Regex("\\s+"))
    if (parts.size != 3) throw XmlException("三元组应为 3 个数字，实际为 \"$text\"")
    return DoubleArray(3) {
        parts[it].toDoubleOrNull() ?: throw XmlException("无法解析数字 \"${parts[it]}\"")
    }
}

fun DoubleArray.fmt(): String = joinToString(" ") { formatNum(it) }

fun formatNum(v: Double): String {
    if (v == 0.0) return "0"
    val a = abs(v)
    if (a >= 1e6 || a < 1e-6) return "%.6e".format(v)
    return "%.6f".format(v).trimEnd('0').trimEnd('.')
}
