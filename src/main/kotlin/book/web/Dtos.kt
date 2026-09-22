package book.web

import book.calib.CalibrationEdge
import book.calib.CalibrationVersion
import book.db.ExportPatchRecord
import book.db.RobotRecord
import book.db.SnapshotRecord
import book.kin.GraphReport
import book.kin.JointValue
import book.kin.PathQueryResult
import book.math.Mat4
import book.urdf.ParseIssue
import kotlinx.serialization.Serializable

@Serializable
data class MatrixDto(val rows: List<List<Double>>) {
    companion object {
        fun of(m: Mat4) = MatrixDto(m.rows().map { row ->
            row.map { v -> if (kotlin.math.abs(v) < 1e-12) 0.0 else v }
        })
    }
}

@Serializable
data class StepDto(
    val edgeId: String,
    val from: String,
    val to: String,
    val reversed: Boolean,
    val explanation: String,
    val jointName: String?,
    val jointValue: Double?,
    val unit: String?,
    val transform: MatrixDto,
)

@Serializable
data class CandidateDto(
    val index: Int,
    val complete: Boolean,
    val problems: List<String>,
    val sources: List<String>,
    val transform: MatrixDto,
    val steps: List<StepDto>,
    val chainSummary: String,
)

@Serializable
data class QueryResponse(
    val from: String,
    val to: String,
    val status: String,
    val chosenIndex: Int?,
    val spreadTranslationMeters: Double?,
    val spreadRotationDeg: Double?,
    val jointValues: List<JointValueDto>,
    val candidates: List<CandidateDto>,
)

@Serializable
data class JointValueDto(
    val jointName: String,
    val value: Double?,
    val unit: String,
    val providedUnit: String?,
    val fromMimic: Boolean,
    val mimicChain: List<String>,
    val status: String,
)

@Serializable
data class IssueDto(val level: String, val code: String, val message: String)

@Serializable
data class FrameDto(val id: String, val name: String, val source: String)

@Serializable
data class EdgeDto(
    val id: String, val a: String, val b: String, val kind: String,
    val label: String, val jointName: String?, val jointType: String?,
    val movable: Boolean, val declaredTwice: Boolean,
)

@Serializable
data class LoopDto(
    val edges: List<String>,
    val residual: MatrixDto,
    val translationError: Double,
    val rotationErrorDeg: Double,
    val consistent: Boolean,
)

@Serializable
data class ContradictionDto(
    val edgeSet: List<String>, val size: Int, val method: String, val note: String,
)

@Serializable
data class ReportDto(
    val components: List<List<String>>,
    val brokenLinks: List<String>,
    val duplicateFrames: List<String>,
    val loops: List<LoopDto>,
    val minimumContradiction: ContradictionDto?,
    val duplicateDeclarations: List<String>,
)

@Serializable
data class CalibEdgeDto(
    val id: String, val kind: String, val jointName: String?,
    val parentFrame: String, val childFrame: String,
    val xyz: List<Double>, val rpyRad: List<Double>, val note: String,
) {
    fun toEdge(): CalibrationEdge =
        CalibrationEdge(id, kind, jointName, parentFrame, childFrame, xyz, rpyRad, note)
    companion object {
        fun of(e: CalibrationEdge) = CalibEdgeDto(
            e.id, e.kind, e.jointName, e.parentFrame, e.childFrame, e.xyz, e.rpyRad, e.note)
    }
}

@Serializable
data class CalibVersionDto(
    val id: String, val robotSerial: String, val version: Int, val lockVersion: Int = 0,
    val note: String,
    val createdAt: String, val validFrom: String, val validUntil: String,
    val draft: Boolean, val approved: Boolean, val edges: List<CalibEdgeDto>,
    val validityStatus: String? = null,
    val validityReason: String? = null,
) {
    companion object {
        fun of(
            v: CalibrationVersion, status: String? = null, reason: String? = null,
            lockVersion: Int = 0,
        ) =
            CalibVersionDto(v.id, v.robotSerial, v.version, lockVersion, v.note, v.createdAt,
                v.validFrom, v.validUntil, v.draft, v.approved,
                v.edges.map { CalibEdgeDto.of(it) }, status, reason)
    }
}

@Serializable
data class SnapshotDto(
    val id: String, val serial: String, val label: String, val frozenAt: String,
    val calibrationVersionId: String?, val jointValues: Map<String, Double>,
    val jointUnits: Map<String, String>, val note: String,
)

@Serializable
data class RobotSummaryDto(val id: String, val serial: String, val name: String, val issueCount: Int)

@Serializable
data class DiffEdgeRow(
    val edgeId: String, val change: String,
    val before: CalibEdgeDto?, val after: CalibEdgeDto?,
)

@Serializable
data class VersionDiffDto(
    val baseVersion: Int?, val targetVersion: Int?,
    val added: List<CalibEdgeDto>,
    val removed: List<CalibEdgeDto>,
    val changed: List<DiffEdgeRow>,
    val windowChanged: Boolean,
    val serialChanged: Boolean,
)

@Serializable
data class PatchDto(
    val id: String, val calibrationVersionId: String,
    val edgeIds: List<String>, val approved: Boolean,
    val createdAt: String, val note: String,
) {
    companion object {
        fun of(p: ExportPatchRecord) = PatchDto(
            p.id, p.calibrationVersionId,
            p.edgeIdsJson.trim('[').trim(']').split(",").map { it.trim().trim('"') }
                .filter { it.isNotEmpty() },
            p.approved, p.createdAt, p.note)
    }
}

@Serializable
data class CompareResponse(
    val snapshot: SnapshotDto,
    val frozenCalib: CalibVersionDto?,
    val otherCalib: CalibVersionDto?,
    val validity: String,
    val queries: List<QueryResponse>,
)

@Serializable
data class ErrorDto(val error: String, val detail: String? = null)
