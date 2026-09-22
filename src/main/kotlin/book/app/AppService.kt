package book.app

import book.calib.CalibRules
import book.calib.CalibrationEdge
import book.calib.CalibrationVersion
import book.calib.UrdfExporter
import book.db.Database
import book.db.ExportPatchRecord
import book.db.RobotRecord
import book.db.SnapshotRecord
import book.kin.GraphAnalyzer
import book.kin.GraphBuilder
import book.kin.KinGraph
import book.kin.PathQueryResult
import book.urdf.UrdfParser
import book.web.*
import kotlinx.serialization.json.Json
import kotlinx.serialization.encodeToString
import kotlinx.serialization.decodeFromString
import java.time.Instant
import java.util.UUID

class AppService(val db: Database) {
    private val json = Json { ignoreUnknownKeys = true }

    fun loadRobot(serial: String): RobotContext? {
        val rec = db.findRobotBySerial(serial) ?: return null
        val model = UrdfParser.parse(rec.urdfXml)
        return RobotContext(rec, model)
    }

    fun listRobots(): List<RobotSummaryDto> = db.listRobots().map { rec ->
        val model = UrdfParser.parse(rec.urdfXml)
        RobotSummaryDto(rec.id, rec.serial, rec.name, model.issues.size)
    }

    /** 选择某时刻生效的标定版本（同序列号 + 窗口内 + 已批准非草稿）。 */
    fun effectiveCalibration(serial: String, at: Instant): CalibrationVersion? =
        db.listRobots().firstOrNull { it.serial == serial }?.let { robot ->
            db.listCalibrationVersions(robot.id)
                .filter { !it.draft && it.approved }
                .filter { CalibRules.evaluate(it, serial, at).effective }
                .maxByOrNull { it.version }
        }

    fun graphFor(serial: String, at: Instant, calibrationId: String?): GraphBundle {
        val ctx = loadRobot(serial) ?: error("机器人不存在: $serial")
        val calib = if (calibrationId != null) db.getCalibrationVersion(calibrationId)
        else effectiveCalibration(serial, at)
        val graph = GraphBuilder.build(ctx.model, calib)
        val report = GraphAnalyzer(graph).report()
        return GraphBundle(ctx, graph, report, calib)
    }

    fun query(
        serial: String, from: String, to: String,
        values: Map<String, Double>, units: Map<String, String>,
        at: Instant, calibrationId: String?,
    ): Pair<PathQueryResult, List<book.kin.JointValue>> {
        val (_, graph, _, _) = graphFor(serial, at, calibrationId)
        val resolved = graph.resolveJointValues(values, units)
        val result = graph.query(from, to, values, units)
        return result to resolved.values.toList()
    }

    fun freezeSnapshot(
        serial: String, label: String, values: Map<String, Double>,
        units: Map<String, String>, calibrationVersionId: String?, note: String,
    ): SnapshotRecord {
        val robot = db.findRobotBySerial(serial) ?: error("机器人不存在: $serial")
        val rec = SnapshotRecord(
            id = UUID.randomUUID().toString(),
            robotId = robot.id, serial = serial, label = label,
            frozenAt = Instant.now().toString(),
            calibrationVersionId = calibrationVersionId,
            jointValuesJson = json.encodeToString(values),
            jointUnitsJson = json.encodeToString(units),
            note = note,
        )
        db.insertSnapshot(rec)
        return rec
    }

    /** 冻结快照后，用另一套标定重新计算同一批 frame 查询并对比。 */
    fun compareSnapshot(
        snapshotId: String, otherCalibrationId: String,
        queryPairs: List<Pair<String, String>>,
    ): CompareResponse {
        val snap = db.getSnapshot(snapshotId) ?: error("快照不存在: $snapshotId")
        val values: Map<String, Double> = json.decodeFromString(snap.jointValuesJson)
        val units: Map<String, String> = json.decodeFromString(snap.jointUnitsJson)
        val otherCalib = db.getCalibrationVersion(otherCalibrationId)
            ?: error("标定版本不存在: $otherCalibrationId")
        val frozenCalib = snap.calibrationVersionId?.let { db.getCalibrationVersion(it) }
        val at = CalibRules.parseTime(snap.frozenAt) ?: Instant.now()
        val validity = CalibRules.evaluate(otherCalib, snap.serial, at).let {
            it.status.name + "：" + it.reason
        }

        val otherBundle = graphFor(snap.serial, at, otherCalibrationId)
        val queryResults = queryPairs.map { (a, b) ->
            otherBundle.graph.query(a, b, values, units)
        }
        return CompareResponse(
            snapshot = SnapshotDto(snap.id, snap.serial, snap.label, snap.frozenAt,
                snap.calibrationVersionId, values, units, snap.note),
            frozenCalib = frozenCalib?.let { CalibVersionDto.of(it) },
            otherCalib = CalibVersionDto.of(
                otherCalib,
                CalibRules.evaluate(otherCalib, snap.serial, at).status.name,
                CalibRules.evaluate(otherCalib, snap.serial, at).reason,
            ),
            validity = validity,
            queries = queryResults.mapIndexed { i, q -> toQueryDto(q, otherBundle.graph, values, units) },
        )
    }

    fun versionDiff(serial: String, baseVersion: Int?, targetVersion: Int): VersionDiffDto {
        val robot = db.findRobotBySerial(serial) ?: error("机器人不存在: $serial")
        val versions = db.listCalibrationVersions(robot.id)
        val target = versions.first { it.version == targetVersion }
        val base = baseVersion?.let { v -> versions.firstOrNull { it.version == v } }
            ?: versions.filter { it.version < targetVersion }.maxByOrNull { it.version }
        val baseEdges = base?.edges.orEmpty().associateBy { it.id }
        val targetEdges = target.edges.associateBy { it.id }
        val added = target.edges.filter { it.id !in baseEdges }
        val removed = base?.edges.orEmpty().filter { it.id !in targetEdges }
        val changed = target.edges.mapNotNull { e ->
            val b = baseEdges[e.id] ?: return@mapNotNull null
            if (b == e) null else DiffEdgeRow(
                e.id, "changed", CalibEdgeDto.of(b), CalibEdgeDto.of(e))
        }
        return VersionDiffDto(
            base?.version, target.version,
            added.map { CalibEdgeDto.of(it) },
            removed.map { CalibEdgeDto.of(it) },
            changed,
            windowChanged = base?.let { it.validFrom != target.validFrom || it.validUntil != target.validUntil } ?: false,
            serialChanged = base?.robotSerial != target.robotSerial,
        )
    }

    fun createDraft(
        serial: String, note: String, validFrom: String, validUntil: String,
        edges: List<CalibrationEdge>,
    ): CalibrationVersion {
        val robot = db.findRobotBySerial(serial) ?: error("机器人不存在: $serial")
        val v = CalibrationVersion(
            id = UUID.randomUUID().toString(),
            robotSerial = serial,
            version = db.nextCalibrationVersionNumber(robot.id),
            note = note, createdAt = Instant.now().toString(),
            validFrom = validFrom, validUntil = validUntil,
            draft = true, approved = false, edges = edges,
        )
        db.insertCalibrationVersion(v, robot.id)
        return v
    }

    fun updateDraft(
        versionId: String, expectedLockVersion: Int, note: String,
        approved: Boolean, edges: List<CalibEdgeDto>,
    ): CalibrationVersion =
        db.updateCalibrationVersion(
            versionId, expectedLockVersion, note,
            draft = !approved, approved = approved,
            edges = edges.map { it.toEdge() },
        )

    fun approveVersion(versionId: String): CalibrationVersion {
        val v = db.getCalibrationVersion(versionId) ?: error("标定版本不存在")
        val lock = db.lockVersion(versionId)
        return db.updateCalibrationVersion(versionId, lock, v.note, false, true, v.edges)
    }

    fun createExportPatch(
        serial: String, calibrationVersionId: String, edgeIds: List<String>, note: String,
    ): ExportPatchRecord {
        val robot = db.findRobotBySerial(serial) ?: error("机器人不存在: $serial")
        val rec = ExportPatchRecord(
            id = UUID.randomUUID().toString(), robotId = robot.id,
            calibrationVersionId = calibrationVersionId,
            edgeIdsJson = json.encodeToString(edgeIds),
            approved = false, createdAt = Instant.now().toString(), note = note,
        )
        db.insertExportPatch(rec)
        return rec
    }

    fun exportWithApprovedPatches(serial: String, calibrationVersionId: String): UrdfExporter.ExportResult {
        val ctx = loadRobot(serial) ?: error("机器人不存在: $serial")
        val calib = db.getCalibrationVersion(calibrationVersionId)
            ?: error("标定版本不存在")
        val approvedEdgeIds = db.listExportPatches(ctx.record.id)
            .filter { it.approved && it.calibrationVersionId == calibrationVersionId }
            .flatMap { PatchDto.of(it).edgeIds }
            .toSet()
        return UrdfExporter.applyApproved(ctx.model, calib, approvedEdgeIds)
    }

    fun toQueryDto(
        result: PathQueryResult, graph: KinGraph,
        values: Map<String, Double>, units: Map<String, String>,
    ): QueryResponse {
        val resolved = graph.resolveJointValues(values, units)
        return QueryResponse(
            from = result.from, to = result.to, status = result.status,
            chosenIndex = result.chosen?.index,
            spreadTranslationMeters = result.spread?.translationMeters,
            spreadRotationDeg = result.spread?.let { Math.toDegrees(it.rotationRadians) },
            jointValues = resolved.values.map {
                JointValueDto(it.jointName, it.value, it.unit, it.providedUnit,
                    it.fromMimic, it.mimicChain, it.status)
            },
            candidates = result.candidates.map { c ->
                CandidateDto(
                    c.index, c.complete, c.problems, c.edgeSources,
                    MatrixDto.of(c.transform),
                    c.steps.map { s ->
                        StepDto(s.edgeId, s.fromFrame, s.toFrame, s.reversed,
                            s.explanation, s.jointName, s.jointValue, s.unit,
                            MatrixDto.of(s.transform))
                    },
                    chainSummary = c.steps.joinToString(" → ") { "${it.fromFrame}→${it.toFrame}" },
                )
            },
        )
    }
}

data class RobotContext(val record: RobotRecord, val model: book.urdf.UrdfModel)

data class GraphBundle(
    val context: RobotContext,
    val graph: KinGraph,
    val report: book.kin.GraphReport,
    val calibration: CalibrationVersion?,
)
