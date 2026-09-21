package com.example.jointbook

import kotlinx.serialization.Serializable
import org.w3c.dom.Element

enum class AngleUnit { RADIAN, DEGREE }
enum class LinearUnit { METER, MILLIMETER }
enum class JointKind { FIXED, REVOLUTE, CONTINUOUS, PRISMATIC, PLANAR, FLOATING, UNKNOWN }
enum class CalibrationApplicability { ACTIVE, BEFORE_START, AT_OR_AFTER_END, SERIAL_MISMATCH, MISSING }
enum class ValueStatus { PROVIDED, MIMIC, MISSING, MIMIC_CYCLE, UNSUPPORTED }
enum class EdgeDirection { FORWARD, REVERSED }

@Serializable
data class Origin(
    val xyz: Vector3 = listOf(0.0, 0.0, 0.0),
    val rpy: Vector3 = listOf(0.0, 0.0, 0.0),
    val angleUnit: AngleUnit = AngleUnit.RADIAN,
    val linearUnit: LinearUnit = LinearUnit.METER
) {
    fun matrix(): Matrix4 {
        val scale = if (linearUnit == LinearUnit.MILLIMETER) 0.001 else 1.0
        val angleScale = if (angleUnit == AngleUnit.DEGREE) Math.PI / 180.0 else 1.0
        return Math3d.xyzRpy(xyz.map { it * scale }, rpy.map { it * angleScale })
    }
}

@Serializable
data class JointLimit(val lower: Double?, val upper: Double?, val effort: Double?, val velocity: Double?)

@Serializable
data class MimicRule(val joint: String, val multiplier: Double = 1.0, val offset: Double = 0.0)

@Serializable
data class InertialValue(
    val origin: Origin = Origin(),
    val mass: Double? = null,
    val ixx: Double? = null,
    val ixy: Double? = null,
    val ixz: Double? = null,
    val iyy: Double? = null,
    val iyz: Double? = null,
    val izz: Double? = null
)

@Serializable
data class GeometryDescription(
    val kind: String,
    val attributes: Map<String, String> = emptyMap(),
    val text: String? = null
)

@Serializable
data class VisualLike(val origin: Origin = Origin(), val geometry: GeometryDescription? = null, val name: String? = null)

@Serializable
data class RawDescriptor(val tag: String, val attributes: Map<String, String>, val order: Int)

@Serializable
data class Link(
    val name: String,
    val occurrence: Int,
    val inertial: InertialValue? = null,
    val visuals: List<VisualLike> = emptyList(),
    val collisions: List<VisualLike> = emptyList(),
    val unknown: List<RawDescriptor> = emptyList()
)

@Serializable
data class Joint(
    val name: String,
    val occurrence: Int,
    val type: JointKind,
    val parent: String,
    val child: String,
    val origin: Origin = Origin(),
    val axis: Vector3 = listOf(1.0, 0.0, 0.0),
    val limit: JointLimit? = null,
    val mimic: MimicRule? = null,
    val unknown: List<RawDescriptor> = emptyList()
)

data class UrdfRobot(
    val id: String,
    val name: String,
    val version: Int,
    val xml: String,
    val root: Element,
    val links: List<Link>,
    val joints: List<Joint>,
    val unknownRobotChildren: List<RawDescriptor>
) {
    fun linkOccurrences(name: String) = links.filter { it.name == name }
    fun jointOccurrences(name: String) = joints.filter { it.name == name }
}

@Serializable
data class CalibratedOrigin(
    val parent: String,
    val child: String,
    val origin: Origin,
    val replacesJoint: String? = null,
    val note: String? = null
)

@Serializable
data class CalibrationVersion(
    val id: String,
    val robotId: String,
    val version: Int,
    val serialNumber: String,
    val validFrom: Long? = null,
    val validUntil: Long? = null,
    val transforms: List<CalibratedOrigin> = emptyList(),
    val note: String = ""
) {
    fun applicability(serial: String, at: Long?): CalibrationApplicability = when {
        serial != serialNumber -> CalibrationApplicability.SERIAL_MISMATCH
        validFrom != null && at != null && at < validFrom -> CalibrationApplicability.BEFORE_START
        validUntil != null && at != null && at >= validUntil -> CalibrationApplicability.AT_OR_AFTER_END
        else -> CalibrationApplicability.ACTIVE
    }
}

@Serializable
data class JointValue(val value: Double, val angleUnit: AngleUnit = AngleUnit.RADIAN, val linearUnit: LinearUnit = LinearUnit.METER)

@Serializable
data class FieldSnapshot(
    val id: String,
    val robotId: String,
    val serialNumber: String,
    val at: Long,
    val frozenCalibrationId: String? = null,
    val jointValues: Map<String, JointValue> = emptyMap(),
    val note: String = ""
)

@Serializable
data class SourceEntry(val kind: String, val id: String, val detail: String)

@Serializable
data class ResolvedJointValue(
    val joint: String,
    val value: Double?,
    val unit: String,
    val status: ValueStatus,
    val sources: List<SourceEntry>
)

@Serializable
data class PathEdge(
    val edgeId: String,
    val joint: String,
    val from: String,
    val to: String,
    val direction: EdgeDirection,
    val transform: Matrix4,
    val jointValue: ResolvedJointValue,
    val sources: List<SourceEntry>
)

@Serializable
data class TransformCandidate(
    val order: Int,
    val frames: List<String>,
    val edges: List<PathEdge>,
    val transform: Matrix4,
    val error: Double,
    val truncated: Boolean = false
)

@Serializable
data class FrameQueryResult(
    val from: String,
    val to: String,
    val status: String,
    val completeChain: List<String> = emptyList(),
    val candidates: List<TransformCandidate> = emptyList(),
    val jointValues: List<ResolvedJointValue> = emptyList(),
    val units: Map<String, String> = emptyMap(),
    val transform: Matrix4? = null,
    val warnings: List<String> = emptyList(),
    val sources: List<SourceEntry> = emptyList()
)

@Serializable
data class CycleResidual(
    val frames: List<String>,
    val edges: List<String>,
    val residual: Double,
    val consistent: Boolean,
    val minimalContradiction: Boolean = false,
    val transform: Matrix4
)

@Serializable
data class GraphAnalysis(
    val nodes: List<GraphNode>,
    val edges: List<GraphEdge>,
    val cycles: List<CycleResidual>,
    val minimalContradictionEdges: List<List<String>>,
    val duplicateFrames: List<String>,
    val brokenEdges: List<String>,
    val mimicCycles: List<List<String>>,
    val duplicateDeclarations: List<DuplicateDeclaration>
)

@Serializable
data class GraphNode(val id: String, val label: String, val occurrence: Int, val missing: Boolean = false, val duplicate: Boolean = false)

@Serializable
data class GraphEdge(
    val id: String,
    val label: String,
    val parent: String,
    val child: String,
    val type: String,
    val activeSource: String,
    val duplicateDeclaration: Boolean = false,
    val broken: Boolean = false
)

@Serializable
data class DuplicateDeclaration(val edge: String, val urdf: SourceEntry, val calibration: SourceEntry)
