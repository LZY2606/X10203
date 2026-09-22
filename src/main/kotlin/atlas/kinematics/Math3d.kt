package atlas.kinematics

import atlas.urdf.Origin
import atlas.urdf.Rpy
import atlas.urdf.Xyz
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

/** Row-major 4x4 homogeneous transform. */
typealias Mat4 = DoubleArray

object Mat {
    fun identity(): Mat4 = doubleArrayOf(
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0
    )

    fun mul(a: Mat4, b: Mat4): Mat4 {
        val r = DoubleArray(16)
        for (i in 0..3) for (j in 0..3) {
            var s = 0.0
            for (k in 0..3) s += a[i * 4 + k] * b[k * 4 + j]
            r[i * 4 + j] = s
        }
        return r
    }

    fun compose(vararg ms: Mat4): Mat4 = ms.fold(identity()) { acc, m -> mul(acc, m) }

    fun translation(x: Double, y: Double, z: Double): Mat4 {
        val m = identity()
        m[3] = x; m[7] = y; m[11] = z
        return m
    }

    fun rotX(a: Double): Mat4 {
        val c = cos(a); val s = sin(a)
        return doubleArrayOf(
            1.0, 0.0, 0.0, 0.0,
            0.0, c, -s, 0.0,
            0.0, s, c, 0.0,
            0.0, 0.0, 0.0, 1.0
        )
    }

    fun rotY(a: Double): Mat4 {
        val c = cos(a); val s = sin(a)
        return doubleArrayOf(
            c, 0.0, s, 0.0,
            0.0, 1.0, 0.0, 0.0,
            -s, 0.0, c, 0.0,
            0.0, 0.0, 0.0, 1.0
        )
    }

    fun rotZ(a: Double): Mat4 {
        val c = cos(a); val s = sin(a)
        return doubleArrayOf(
            c, -s, 0.0, 0.0,
            s, c, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0
        )
    }

    /** Rotation about an arbitrary unit axis (Rodrigues). */
    fun rotAxis(axis: Xyz, angle: Double): Mat4 {
        val len = sqrt(axis.x * axis.x + axis.y * axis.y + axis.z * axis.z)
        if (len < 1e-12) return identity()
        val x = axis.x / len; val y = axis.y / len; val z = axis.z / len
        val c = cos(angle); val s = sin(angle); val t = 1 - c
        return doubleArrayOf(
            t * x * x + c, t * x * y - s * z, t * x * z + s * y, 0.0,
            t * x * y + s * z, t * y * y + c, t * y * z - s * x, 0.0,
            t * x * z - s * y, t * y * z + s * x, t * z * z + c, 0.0,
            0.0, 0.0, 0.0, 1.0
        )
    }

    /** URDF convention: fixed-axis roll-pitch-yaw, R = Rz(yaw) * Ry(pitch) * Rx(roll). */
    fun fromOrigin(o: Origin): Mat4 =
        compose(
            translation(o.xyz.x, o.xyz.y, o.xyz.z),
            rotZ(o.rpy.yaw), rotY(o.rpy.pitch), rotX(o.rpy.roll)
        )

    fun fromXyzRpy(xyz: Xyz, rpy: Rpy): Mat4 = fromOrigin(Origin(xyz, rpy))

    /** Inverse of a rigid transform (rotation + translation). */
    fun inverseRigid(m: Mat4): Mat4 {
        val r = identity()
        for (i in 0..2) for (j in 0..2) r[i * 4 + j] = m[j * 4 + i]
        val tx = m[3]; val ty = m[7]; val tz = m[11]
        r[3] = -(r[0] * tx + r[1] * ty + r[2] * tz)
        r[7] = -(r[4] * tx + r[5] * ty + r[6] * tz)
        r[11] = -(r[8] * tx + r[9] * ty + r[10] * tz)
        return r
    }

    /** Max absolute element difference. */
    fun maxDiff(a: Mat4, b: Mat4): Double {
        var d = 0.0
        for (i in 0..15) {
            val e = kotlin.math.abs(a[i] - b[i])
            if (e > d) d = e
        }
        return d
    }

    /** Deviation of m from identity: rotation part + translation norm. */
    fun residualOf(m: Mat4): Double {
        var rot = 0.0
        val id = identity()
        for (i in 0..2) for (j in 0..2) {
            val e = kotlin.math.abs(m[i * 4 + j] - id[i * 4 + j])
            if (e > rot) rot = e
        }
        val t = sqrt(m[3] * m[3] + m[7] * m[7] + m[11] * m[11])
        return rot + t
    }

    fun applyPoint(m: Mat4, x: Double, y: Double, z: Double): Triple<Double, Double, Double> =
        Triple(
            m[0] * x + m[1] * y + m[2] * z + m[3],
            m[4] * x + m[5] * y + m[6] * z + m[7],
            m[8] * x + m[9] * y + m[10] * z + m[11]
        )
}
