package coordbook

import kotlinx.serialization.Serializable

@Serializable
data class RobotSummaryDto(
    val id: Long,
    val name: String,
    val serial: String,
    val revision: Int,
    val linkCount: Int,
    val jointCount: Int,
)

@Serializable
data class IssueDto(
    val severity: String,
    val code: String,
    val message: String,
    val refId: String? = null,
)

@Serializable
data class PoseDto(val xyz: List<Double>, val rpyRadians: List<Double>)

@Serializable
data class LinkDto(
    val id: String,
    val name: String,
    val hasInertial: Boolean,
    val massKg: Double? = null,
    val visuals: Int,
    val collisions: Int,
)

@Serializable
data class JointDto(
    val id: String,
    val name: String,
    val type: String,
    val parent: String,
    val child: String,
    val origin: PoseDto,
    val axis: List<Double>,
    val unit: String,
    val limitLower: Double? = null,
    val limitUpper: Double? = null,
    val mimicJoint: String? = null,
    val mimicMultiplier: Double? = null,
    val mimicOffset: Double? = null,
)

@Serializable
data class RobotDetailDto(
    val id: Long,
    val name: String,
    val serial: String,
    val revision: Int,
    val links: List<LinkDto>,
    val joints: List<JointDto>,
    val issues: List<IssueDto>,
    val graphIssues: List<IssueDto>,
    val components: List<List<String>>,
)

@Serializable
data class SnapshotDto(
    val id: Long,
    val serial: String,
    val frozenAt: String,
    val jointValues: Map<String, Double>,
    val note: String? = null,
)

@Serializable
data class CalibPatchDto(
    val id: Long,
    val versionId: Long,
    val kind: String,
    val targetJoint: String? = null,
    val parentFrame: String? = null,
    val childFrame: String? = null,
    val xyz: List<Double>? = null,
    val rpyRadians: List<Double>? = null,
    val status: String,
    val note: String? = null,
)

@Serializable
data class CalibVersionDto(
    val id: Long,
    val revision: Int,
    val serialScope: String,
    val validFrom: String,
    val validUntil: String,
    val note: String? = null,
    val patches: List<CalibPatchDto>,
)

@Serializable
data class EffectiveDto(
    val state: String,
    val reason: String,
    val versionId: Long? = null,
    val revision: Int? = null,
    val appliedPatchIds: List<Long>,
)

@Serializable
data class ChainStepDto(
    val fromFrame: String,
    val toFrame: String,
    val edgeLabel: String,
    val edgeKind: String,
    val jointName: String? = null,
    val jointType: String? = null,
    val jointValueRadOrM: Double? = null,
    val jointValueDeg: Double? = null,
    val unit: String? = null,
    val valueOrigin: String? = null,
    val transform: List<List<Double>>,
)

@Serializable
data class PathCandidateDto(
    val index: Int,
    val frames: List<String>,
    val steps: List<ChainStepDto>,
    val transform: List<List<Double>>,
)

@Serializable
data class MissingJointDto(val jointName: String, val reason: String)

@Serializable
data class QueryResponseDto(
    val ok: Boolean,
    val error: String? = null,
    val ambiguousFrameIds: List<String>? = null,
    val fromFrame: String,
    val toFrame: String,
    val effective: EffectiveDto,
    val candidates: List<PathCandidateDto>,
    val candidateCount: Int,
    val consistent: Boolean,
    val maxTranslationErrorM: Double,
    val maxRotationErrorRad: Double,
    val consensusTransform: List<List<Double>>? = null,
    val missingJoint: MissingJointDto? = null,
)

@Serializable
data class CycleResidualDto(
    val frames: List<String>,
    val edgeLabels: List<String>,
    val closureTransform: List<List<Double>>,
    val translationErrorM: Double,
    val rotationErrorRad: Double,
    val consistent: Boolean,
)

@Serializable
data class CycleResponseDto(
    val hasCycle: Boolean,
    val inconsistent: Boolean,
    val note: String,
    val residuals: List<CycleResidualDto>,
    val minimalContradictingEdges: List<String>,
    val spanningTreeEdgeIds: List<String>,
)

@Serializable
data class VersionDiffDto(
    val baseRevision: Int,
    val targetRevision: Int,
    val added: List<CalibPatchDto>,
    val removed: List<CalibPatchDto>,
    val changedStatus: List<StatusChangeDto>,
)

@Serializable
data class StatusChangeDto(
    val patchId: Long,
    val from: String,
    val to: String,
    val kind: String,
)

@Serializable
data class ExportResponseDto(
    val xml: String,
    val appliedPatches: List<AppliedPatchDto>,
    val warning: String,
)

@Serializable
data class AppliedPatchDto(val patchId: Long, val jointName: String, val changed: List<String>)

// ---- 请求体 ----

@Serializable
data class ImportRobotRequest(val name: String, val serial: String, val urdf: String)

@Serializable
data class UpdateUrdfRequest(val urdf: String, val expectedRevision: Int)

@Serializable
data class CreateSnapshotRequest(
    val serial: String,
    val frozenAt: String,
    val jointValues: Map<String, Double>,
    val note: String? = null,
)

@Serializable
data class CreateVersionRequest(
    val serialScope: String,
    val validFrom: String,
    val validUntil: String,
    val note: String? = null,
)

@Serializable
data class CreatePatchRequest(
    val versionId: Long,
    val kind: String,
    val targetJoint: String? = null,
    val parentFrame: String? = null,
    val childFrame: String? = null,
    val xyz: List<Double>? = null,
    val rpyRadians: List<Double>? = null,
    val note: String? = null,
)

@Serializable
data class ApproveRequest(val status: String, val expectedRevision: Int)

@Serializable
data class QueryRequest(
    val from: String,
    val to: String,
    val at: String,
    val snapshotId: Long? = null,
    val calibVersionId: Long? = null,
    val jointValuesDeg: Map<String, Double>? = null,
    val jointValuesRad: Map<String, Double>? = null,
)

@Serializable
data class DiffRequest(val baseVersionId: Long, val targetVersionId: Long)

object DtoConversions {
    fun pose(p: Pose) = PoseDto(p.xyz.toList(), p.rpyRadians.toList())

    fun issue(i: DocumentIssue) = IssueDto(i.severity.name, i.code, i.message, i.refId)

    fun link(l: LinkNode) = LinkDto(
        l.id, l.name, l.inertial != null, l.inertial?.massKg, l.visuals.size, l.collisions.size,
    )

    fun joint(j: JointNode) = JointDto(
        j.id, j.name, j.type.urdfName, j.parentLink, j.childLink,
        pose(j.origin), j.axis.toList(), j.type.unit,
        j.limit?.lowerRadOrM, j.limit?.upperRadOrM,
        j.mimic?.jointName, j.mimic?.multiplier, j.mimic?.offset,
    )

    fun patch(p: CalibPatch) = CalibPatchDto(
        p.id, p.versionId, p.kind.name, p.targetJoint, p.parentFrame, p.childFrame,
        p.xyz?.toList(), p.rpyRadians?.toList(), p.status.name, p.note,
    )

    fun version(v: CalibrationVersion, patches: List<CalibPatch>) = CalibVersionDto(
        v.id, v.revision, v.serialScope, v.validFromIso, v.validUntilIso, v.note,
        patches.filter { it.versionId == v.id }.map(::patch),
    )

    fun snapshot(s: Snapshot) = SnapshotDto(s.id, s.serial, s.frozenAtIso, s.jointValues, s.note)
}
