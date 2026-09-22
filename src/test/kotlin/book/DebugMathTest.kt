package book
import book.math.Mat4
import book.math.Vec3
import kotlin.test.Test
class DebugMathTest {
  @Test fun debug() {
    val t = Mat4.fromXyzRpy(Vec3(1.0,0.0,0.0), Vec3.ZERO)
    println("T rows:"); t.rows().forEach { println(it.joinToString(" ") { v -> "%.3f".format(v) }) }
    val ti = t.inverse()
    println("T^-1 rows:"); ti.rows().forEach { println(it.joinToString(" ") { v -> "%.3f".format(v) }) }
    println("T*T^-1:"); (t*ti).rows().forEach { println(it.joinToString(" ") { v -> "%.3f".format(v) }) }
    val p = t.applyPoint(Vec3.ZERO); println("T*0 = $p")
    val pi = ti.applyPoint(Vec3.ZERO); println("T^-1*0 = $pi")
    println("loop check: T^-1 * T translation should be 0")
    ((t.inverse()) * t).rows().forEach { println(it.joinToString(" ") { v -> "%.3f".format(v) }) }
  }
}
