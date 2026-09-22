package book.web

import book.app.AppService
import book.calib.CalibRules
import book.calib.UrdfExporter
import io.ktor.http.*
import io.ktor.serialization.kotlinx.json.*
import io.ktor.server.application.*
import io.ktor.server.engine.*
import io.ktor.server.netty.*
import io.ktor.server.plugins.contentnegotiation.*
import io.ktor.server.request.*
import io.ktor.server.response.*
import io.ktor.server.routing.*
import kotlinx.serialization.Serializable
import java.time.Instant

@Serializable
data class QueryRequest(
    val serial: String,
    val from: String,
    val to: String,
    val at: String? = null,
    val calibrationVersionId: String? = null,
    val values: Map<String, Double> = emptyMap(),
    val units: Map<String, String> = emptyMap(),
)

@Serializable
data class SnapshotRequest(
    val serial: String, val label: String,
    val values: Map<String, Double> = emptyMap(),
    val units: Map<String, String> = emptyMap(),
    val calibrationVersionId: String? = null,
    val note: String = "",
)

@Serializable
data class CompareRequest(
    val snapshotId: String, val otherCalibrationVersionId: String,
    val pairs: List<FramePair> = emptyList(),
)

@Serializable
data class FramePair(val from: String, val to: String)

@Serializable
data class DraftRequest(
    val serial: String, val note: String,
    val validFrom: String, val validUntil: String,
    val edges: List<CalibEdgeDto>,
    val baseVersionId: String? = null,
    val expectedLockVersion: Int? = null,
)

@Serializable
data class PatchRequest(
    val serial: String, val calibrationVersionId: String,
    val edgeIds: List<String>, val note: String = "",
)

class WebServer(private val service: AppService) {
    fun start(host: String, port: Int) {
        embeddedServer(Netty, host = host, port = port) {
            install(ContentNegotiation) { json() }
            routing {
                get("/") {
                    val html = javaClass.getResourceAsStream("/index.html")?.bufferedReader()
                        ?.use { it.readText() }
                        ?: error("index.html 未找到")
                    call.respondText(html, ContentType.Text.Html)
                }
                get("/app.js") {
                    val js = javaClass.getResourceAsStream("/app.js")?.bufferedReader()
                        ?.use { it.readText() }
                        ?: return@get call.respond(HttpStatusCode.NotFound, "app.js missing")
                    call.respondText(js, ContentType("application", "javascript"))
                }
                get("/api/robots") {
                    call.respond(service.listRobots())
                }
                get("/api/robot/{serial}") {
                    val serial = call.parameters["serial"]!!
                    val at = call.request.queryParameters["at"]?.let { Instant.parse(it) } ?: Instant.now()
                    val calibId = call.request.queryParameters["calibrationVersionId"]
                    val bundle = service.graphFor(serial, at, calibId)
                    val ctx = bundle.context
                    val locks = service.db.listCalibrationVersions(ctx.record.id)
                        .associate { it.id to service.db.lockVersion(it.id) }
                    val validity = bundle.calibration?.let {
                        val v = CalibRules.evaluate(it, serial, at)
                        CalibVersionDto.of(it, v.status.name, v.reason, locks[it.id] ?: 0)
                    }
                    call.respond(
                        mapOf(
                            "serial" to ctx.record.serial,
                            "name" to ctx.record.name,
                            "issues" to ctx.model.issues.map {
                                IssueDto(it.level, it.code, it.message)
                            },
                            "frames" to bundle.graph.frameNodes.values.map {
                                FrameDto(it.id, it.name, it.source)
                            },
                            "edges" to bundle.graph.edges.map { e ->
                                EdgeDto(e.id, e.a, e.b, e.kind, e.sourceLabel,
                                    e.joint?.name, e.joint?.type?.urdfName,
                                    e.joint?.isMovable() ?: false, e.declaredTwice)
                            },
                            "report" to ReportDto(
                                bundle.report.components,
                                bundle.report.brokenLinks,
                                bundle.report.duplicateFrames,
                                bundle.report.loops.map { lp ->
                                    LoopDto(lp.edges, MatrixDto.of(lp.residualTransform),
                                        lp.translationError, Math.toDegrees(lp.rotationErrorRad),
                                        lp.consistent)
                                },
                                bundle.report.minimumContradiction?.let {
                                    ContradictionDto(it.edgeSet, it.size, it.method, it.note)
                                },
                                bundle.report.duplicateDeclarations,
                            ),
                            "activeCalibration" to validity,
                            "calibrationVersions" to service.db.let { db ->
                                db.listCalibrationVersions(ctx.record.id).map { v ->
                                    val ev = CalibRules.evaluate(v, serial, at)
                                    CalibVersionDto.of(v, ev.status.name, ev.reason)
                                }
                            },
                            "snapshots" to service.db.listSnapshots(ctx.record.id).map {
                                SnapshotDto(it.id, it.serial, it.label, it.frozenAt,
                                    it.calibrationVersionId,
                                    kotlinx.serialization.json.Json.decodeFromString(it.jointValuesJson),
                                    kotlinx.serialization.json.Json.decodeFromString(it.jointUnitsJson),
                                    it.note)
                            },
                            "patches" to service.db.listExportPatches(ctx.record.id).map { PatchDto.of(it) },
                        )
                    )
                }
                get("/api/urdf/{serial}") {
                    val serial = call.parameters["serial"]!!
                    val ctx = service.loadRobot(serial) ?: return@get call.respond(
                        HttpStatusCode.NotFound, ErrorDto("机器人不存在", serial))
                    call.respondText(ctx.record.urdfXml, ContentType.Application.Xml)
                }
                post("/api/query") {
                    val req = call.receive<QueryRequest>()
                    val at = req.at?.let { Instant.parse(it) } ?: Instant.now()
                    val (result, resolved) = service.query(
                        req.serial, req.from, req.to, req.values, req.units, at,
                        req.calibrationVersionId)
                    val bundle = service.graphFor(req.serial, at, req.calibrationVersionId)
                    call.respond(service.toQueryDto(result, bundle.graph, req.values, req.units))
                }
                post("/api/snapshots") {
                    val req = call.receive<SnapshotRequest>()
                    val rec = service.freezeSnapshot(
                        req.serial, req.label, req.values, req.units,
                        req.calibrationVersionId, req.note)
                    call.respond(HttpStatusCode.Created, SnapshotDto(
                        rec.id, rec.serial, rec.label, rec.frozenAt, rec.calibrationVersionId,
                        req.values, req.units, rec.note))
                }
                post("/api/compare") {
                    val req = call.receive<CompareRequest>()
                    call.respond(service.compareSnapshot(
                        req.snapshotId, req.otherCalibrationVersionId,
                        req.pairs.map { it.from to it.to }))
                }
                get("/api/versions/{serial}") {
                    val serial = call.parameters["serial"]!!
                    val robot = service.db.listRobots().firstOrNull { it.serial == serial }
                        ?: return@get call.respond(HttpStatusCode.NotFound, ErrorDto("机器人不存在"))
                    val at = Instant.now()
                    call.respond(service.db.listCalibrationVersions(robot.id).map {
                        val ev = CalibRules.evaluate(it, serial, at)
                        CalibVersionDto.of(it, ev.status.name, ev.reason,
                            service.db.lockVersion(it.id))
                    })
                }
                get("/api/diff/{serial}") {
                    val serial = call.parameters["serial"]!!
                    val target = call.request.queryParameters["target"]!!.toInt()
                    val base = call.request.queryParameters["base"]?.toInt()
                    call.respond(service.versionDiff(serial, base, target))
                }
                post("/api/versions") {
                    val req = call.receive<DraftRequest>()
                    if (req.baseVersionId != null && req.expectedLockVersion != null) {
                        val updated = service.updateDraft(
                            req.baseVersionId, req.expectedLockVersion, req.note,
                            false, req.edges)
                        call.respond(CalibVersionDto.of(updated))
                    } else {
                        val v = service.createDraft(
                            req.serial, req.note, req.validFrom, req.validUntil,
                            req.edges.map { it.toEdge() })
                        call.respond(HttpStatusCode.Created, CalibVersionDto.of(v))
                    }
                }
                post("/api/versions/{id}/approve") {
                    val v = service.approveVersion(call.parameters["id"]!!)
                    call.respond(CalibVersionDto.of(v))
                }
                post("/api/patches") {
                    val req = call.receive<PatchRequest>()
                    val rec = service.createExportPatch(
                        req.serial, req.calibrationVersionId, req.edgeIds, req.note)
                    call.respond(HttpStatusCode.Created, PatchDto.of(rec))
                }
                post("/api/patches/{id}/approve") {
                    val rec = service.db.approvePatch(call.parameters["id"]!!)
                        ?: return@post call.respond(HttpStatusCode.NotFound, ErrorDto("补丁不存在"))
                    call.respond(PatchDto.of(rec))
                }
                get("/api/export/{serial}") {
                    val serial = call.parameters["serial"]!!
                    val calibId = call.request.queryParameters["calibrationVersionId"]
                        ?: return@get call.respond(
                            HttpStatusCode.BadRequest, ErrorDto("需要 calibrationVersionId"))
                    val result = service.exportWithApprovedPatches(serial, calibId)
                    call.respond(
                        mapOf(
                            "appliedEdges" to result.appliedEdges,
                            "skippedUnapproved" to result.skippedUnapproved,
                            "skippedMissing" to result.skippedMissing,
                            "warnings" to result.warnings,
                            "derived" to true,
                        )
                    )
                }
                get("/api/export/{serial}/xml") {
                    val serial = call.parameters["serial"]!!
                    val calibId = call.request.queryParameters["calibrationVersionId"]
                        ?: return@get call.respond(
                            HttpStatusCode.BadRequest, ErrorDto("需要 calibrationVersionId"))
                    val result: UrdfExporter.ExportResult = service.exportWithApprovedPatches(serial, calibId)
                    call.respondText(result.xml, ContentType.Application.Xml)
                }
            }
        }.start(wait = true)
    }
}
