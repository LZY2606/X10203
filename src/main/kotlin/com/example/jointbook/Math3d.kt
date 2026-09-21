package com.example.jointbook

import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

typealias Matrix4 = List<List<Double>>
typealias Vector3 = List<Double>

object Math3d {
    val identity: Matrix4 = List(4) { row -> List(4) { col -> if (row == col) 1.0 else 0.0 } }

    fun translation(x: Double, y: Double, z: Double): Matrix4 = listOf(
        listOf(1.0, 0.0, 0.0, x),
        listOf(0.0, 1.0, 0.0, y),
        listOf(0.0, 0.0, 1.0, z),
        listOf(0.0, 0.0, 0.0, 1.0)
    )

    fun rotationX(a: Double): Matrix4 {
        val c = cos(a)
        val s = sin(a)
        return listOf(
            listOf(1.0, 0.0, 0.0, 0.0),
            listOf(0.0, c, -s, 0.0),
            listOf(0.0, s, c, 0.0),
            listOf(0.0, 0.0, 0.0, 1.0)
        )
    }

    fun rotationY(a: Double): Matrix4 {
        val c = cos(a)
        val s = sin(a)
        return listOf(
            listOf(c, 0.0, s, 0.0),
            listOf(0.0, 1.0, 0.0, 0.0),
            listOf(-s, 0.0, c, 0.0),
            listOf(0.0, 0.0, 0.0, 1.0)
        )
    }

    fun rotationZ(a: Double): Matrix4 {
        val c = cos(a)
        val s = sin(a)
        return listOf(
            listOf(c, -s, 0.0, 0.0),
            listOf(s, c, 0.0, 0.0),
            listOf(0.0, 0.0, 1.0, 0.0),
            listOf(0.0, 0.0, 0.0, 1.0)
        )
    }

    fun rotationAxis(axis: Vector3, angle: Double): Matrix4 {
        val length = sqrt(axis.sumOf { it * it })
        if (length < 1e-12) return identity
        val (x, y, z) = axis.map { it / length }
        val c = cos(angle)
        val s = sin(angle)
        val d = 1.0 - c
        return listOf(
            listOf(c + x * x * d, x * y * d - z * s, x * z * d + y * s, 0.0),
            listOf(y * x * d + z * s, c + y * y * d, y * z * d - x * s, 0.0),
            listOf(z * x * d - y * s, z * y * d + x * s, c + z * z * d, 0.0),
            listOf(0.0, 0.0, 0.0, 1.0)
        )
    }

    fun xyzRpy(xyz: Vector3, rpy: Vector3): Matrix4 =
        multiply(
            translation(xyz[0], xyz[1], xyz[2]),
            multiply(rotationZ(rpy[2]), multiply(rotationY(rpy[1]), rotationX(rpy[0])))
        )

    fun multiply(a: Matrix4, b: Matrix4): Matrix4 = List(4) { row ->
        List(4) { col ->
            (0..3).sumOf { k -> a[row][k] * b[k][col] }
        }
    }

    fun multiplyAll(transforms: List<Matrix4>): Matrix4 =
        transforms.fold(identity) { acc, value -> multiply(acc, value) }

    fun inverse(t: Matrix4): Matrix4 {
        val rotation = List(3) { row -> List(3) { col -> t[col][row] } }
        val translation = List(3) { row ->
            -(0..2).sumOf { col -> rotation[row][col] * t[col][3] }
        }
        return listOf(
            listOf(rotation[0][0], rotation[0][1], rotation[0][2], translation[0]),
            listOf(rotation[1][0], rotation[1][1], rotation[1][2], translation[1]),
            listOf(rotation[2][0], rotation[2][1], rotation[2][2], translation[2]),
            listOf(0.0, 0.0, 0.0, 1.0)
        )
    }

    fun residual(a: Matrix4, b: Matrix4): Double =
        (0..3).maxOf { row -> (0..3).maxOf { col -> kotlin.math.abs(a[row][col] - b[row][col]) } }
}
