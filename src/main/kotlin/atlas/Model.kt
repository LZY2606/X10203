package atlas

/** 关节类型 */
enum class JointType { FIXED, REVOLUTE, CONTINUOUS, PRISMATIC, FLOATING, PLANAR, UNKNOWN }

data class Mimic(val joint: String, val multiplier: Double = 1.0, val offset: Double = 0.0)

data class Limit(val lower: Double?, val upper: Double?, val effort: Double?, val velocity: Double?)

data class Origin(val xyz: DoubleArray = doubleArrayOf(0.0, 0.0, 0.0),
                  val rpy: DoubleArray = doubleArrayOf(0.0, 0.0, 0.0))

data class Joint(
    val name: String,
    val type: JointType,
    val parent: String,
    val child: String,
    val origin: Origin,
    val axis: DoubleArray = doubleArrayOf(1.0, 0.0, 0.0),
    val limit: Limit? = null,
    val mimic: Mimic? = null,
)

data class Link(val name: String, val hasInertial: Boolean, val hasGeometry: Boolean)

data class RobotModel(
    val name: String,
    val links: List<Link>,
    val joints: List<Joint>,
    val duplicateFrames: List<String>,
)

/** 标定变换（独立版本化） */
data class Calibration(
    val id: Long,
    val robotId: String,
    val serial: String,
    val parentFrame: String,
    val childFrame: String,
    val xyz: DoubleArray,
    val rpy: DoubleArray,
    val version: Int,
    val validFrom: String,   // ISO-8601, 闭区间起点
    val validTo: String,     // ISO-8601, 闭区间终点
    val approved: Boolean,
)

/** 关节值状态 */
sealed class JointState {
    data class Ok(val valueRad: Double, val unit: String, val raw: Double) : JointState()
    data class Missing(val joint: String) : JointState()
    data class MimicCycle(val joints: List<String>) : JointState()
    data class MimicMissing(val joint: String, val ref: String) : JointState()
}

/** 图中的一条有向变换边（可正向或反向使用） */
data class Edge(
    val id: String,          // urdf:joint:<name> 或 cal:<id>@v<version>
    val source: String,      // "urdf" / "calibration"
    val parent: String,
    val child: String,
    val jointName: String?,  // URDF 关节名；标定边为 null
    val jointType: JointType?,
    val constant: Mat4?,     // 固定变换（fixed 关节或标定）
    val axis: DoubleArray?,  // 运动关节轴
    val mimic: Mimic?,
)

data class PathStep(
    val edgeId: String,
    val source: String,
    val from: String,
    val to: String,
    val reversed: Boolean,
    val jointName: String?,
    val jointState: JointState?,
)

data class PathResult(
    val steps: List<PathStep>,
    val matrix: Mat4?,
    val complete: Boolean,
    val problems: List<String>,
    val errorVsConsensus: Double,
)

data class CycleResidual(
    val cycleEdges: List<String>,
    val translationError: Double,
    val rotationErrorDeg: Double,
    val consistent: Boolean,
)

data class PoseQueryResult(
    val from: String,
    val to: String,
    val candidates: List<PathResult>,
    val status: String,          // ok / multiple-candidates / incomplete / disconnected
    val missingJoints: List<String>,
    val units: Map<String, String>,
)
