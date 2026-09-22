package jcb.web

import jcb.AppService
import jcb.calib.CalibrationTransform
import jcb.db.SnapshotRecord
import jcb.kin.CycleResidual
import jcb.kin.FrameQueryResult
import jcb.kin.PoseDiff
import jcb.kin.Transform
import jcb.kin.TransformCandidate
import kotlinx.serialization.Serializable
import java.time.Instant

@Serializable
data class RobotDto(val id: Long, val serial: String, val name: String, val importedAt: String)

@Serializable
data class ImportRequest(val serial: String, val name: String? = null, val urdfXml: String)

@Serializable
data class SnapshotRequest(
    val label: String,
    val takenAt: String? = null,
    val values: Map<String, ValueDto> = emptyMap(),
)

@Serializable
data class ValueDto(val value: Double, val unit: String = "rad")

@Serializable
data class SnapshotDto(
    val id: Long, val label: String, val takenAt: String?,
    val values: Map<String, ValueDto>, val frozen: Boolean, val createdAt: String,
)

@Serializable
data class CalibDto(
    val id: String, val version: Int, val robotSerial: String,
    val parentFrame: String, val childFrame: String,
    val xyz: List<Double>, val rpyRad: List<Double>,
    val note: String?, val validFrom: String?, val validTo: String?,
    val createdAt: String, val status: String, val baseVersion: Int,
)

@Serializable
data class DraftRequest(
    val id: String, val robotSerial: String? = null, val baseVersion: Int = 0,
    val parentFrame: String, val childFrame: String,
    val xyz: List<Double>, val rpyRad: List<Double>,
    val note: String? = null,
    val validFrom: String? = null, val validTo: String? = null,
)

@Serializable
data class ApproveRequest(val version: Int)

@Serializable
data class QueryRequest(
    val from: String, val to: String,
    val snapshotId: Long? = null,
    val calibrationIds: List<String>? = null,
    val values: Map<String, ValueDto> = emptyMap(),
    val toleranceM: Double = 1e-6,
    val toleranceDeg: Double? = null,
    val compareCalibrationIds: List<String>? = null,
    val compareSnapshotId: Long? = null,
)

@Serializable
data class MatrixDto(val rows: List<List<Double>>)

@Serializable
data class StepDto(
    val edgeId: String, val source: String, val from: String, val to: String,
    val jointName: String?, val jointValue: Double?, val unit: String?,
    val matrix: MatrixDto,
)

@Serializable
data class CandidateDto(
    val steps: List<StepDto>, val matrix: MatrixDto,
    val usesCalibration: Boolean, val calibrationIds: List<String>,
    val movableJoints: List<String>,
)

@Serializable
data class EffectiveDto(
    val id: String, val version: Int, val state: String, val effective: Boolean,
    val reason: String, val parentFrame: String, val childFrame: String,
    val validFrom: String?, val validTo: String?,
)

@Serializable
data class CycleDto(
    val edges: List<String>, val frames: List<String>,
    val matrix: MatrixDto,
    val translationResidualM: Double, val rotationResidualDeg: Double,
    val consistent: Boolean, val toleranceM: Double, val toleranceDeg: Double,
)

@Serializable
data class GraphNodeDto(val id: String, val kind: String)
@Serializable
data class GraphLinkDto(val source: String, val target: String, val label: String, val kind: String)
@Serializable
data class GraphDataDto(
    val nodes: List<GraphNodeDto>, val links: List<GraphLinkDto>,
    val issues: List<IssueDto>,
)
@Serializable
data class IssueDto(val kind: String, val severity: String, val message: String, val ref: String?)

@Serializable
data class DiffEntryDto(
    val parent: String, val child: String,
    val oldVersion: Int?, val newVersion: Int?, val changes: List<String>,
)

@Serializable
data class EvaluationDto(
    val selectorLabel: String,
    val from: String, val to: String, val status: String,
    val missingJoints: List<String>,
    val candidates: List<CandidateDto>,
    val spreadTranslationM: Double?, val spreadRotationDeg: Double?,
    val effectiveCalibrations: List<EffectiveDto>,
    val cycles: List<CycleDto>,
    val inconsistentCycles: List<CycleDto>,
    val minContradictingEdgeSets: List<List<String>>,
    val cycleNote: String,
    val issues: List<IssueDto>,
)

@Serializable
data class CompareResponseDto(val base: EvaluationDto, val other: EvaluationDto, val diffs: List<DiffEntryDto>)

fun CalibrationTransform.toDto() = CalibDto(
    id, version, robotSerial, parentFrame, childFrame,
    xyz.toList(), rpyRad.toList(), note,
    validFrom?.toString(), validTo?.toString(), createdAt.toString(), status.name, baseVersion,
)

fun SnapshotRecord.toDto(values: Map<String, ValueDto>) = SnapshotDto(
    id, label, takenAt?.toString(), values, frozen, createdAt.toString()
)

fun Transform.matrixDto() = MatrixDto(toMatrixRows())

fun candidateDto(c: TransformCandidate) = CandidateDto(
    steps = c.steps.map {
        StepDto(
            it.edgeId, it.source, it.from, it.to, it.jointName,
            it.jointValueRadOrM, it.jointUnit, it.transform.matrixDto()
        )
    },
    matrix = c.transform.matrixDto(),
    usesCalibration = c.usesCalibration,
    calibrationIds = c.calibrationIds,
    movableJoints = c.movableJoints,
)

fun cycleDto(c: CycleResidual) = CycleDto(
    edges = c.edges, frames = c.frames, matrix = c.residual.matrixDto(),
    translationResidualM = c.translationResidualM,
    rotationResidualDeg = Math.toDegrees(c.rotationResidualRad),
    consistent = c.consistent,
    toleranceM = c.toleranceM,
    toleranceDeg = Math.toDegrees(c.toleranceRad),
)

fun statusName(q: FrameQueryResult): String = when (val s = q.status) {
    is jcb.kin.QueryStatus.Ok -> "OK"
    is jcb.kin.QueryStatus.MissingJointValues -> "MISSING_JOINT_VALUES"
    is jcb.kin.QueryStatus.Unreachable -> "UNREACHABLE"
}

fun missingJoints(q: FrameQueryResult): List<String> =
    (q.status as? jcb.kin.QueryStatus.MissingJointValues)?.joints ?: emptyList()

fun spreadDeg(d: PoseDiff?) = d?.let { Math.toDegrees(it.rotationRadians) }
