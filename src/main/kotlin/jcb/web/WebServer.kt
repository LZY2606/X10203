package jcb.web

import io.ktor.http.ContentType
import io.ktor.http.HttpStatusCode
import io.ktor.server.application.call
import io.ktor.server.application.install
import io.ktor.server.engine.embeddedServer
import io.ktor.server.netty.Netty
import io.ktor.server.plugins.contentnegotiation.ContentNegotiation
import io.ktor.serialization.kotlinx.json.json
import io.ktor.server.plugins.statuspages.StatusPages
import io.ktor.server.request.receive
import io.ktor.server.response.respond
import io.ktor.server.response.respondText
import io.ktor.server.routing.delete
import io.ktor.server.routing.get
import io.ktor.server.routing.post
import io.ktor.server.routing.routing
import jcb.AppService
import jcb.JointValueInput
import jcb.EvaluationResult
import jcb.VersionConflictException
import jcb.db.Database
import jcb.urdf.UrdfParser
import kotlinx.serialization.json.Json
import java.time.Instant

class WebServer(private val dbPath: String? = null) {
    private val db = Database(dbPath ?: "jcb.sqlite")
    val service = AppService(db)

    fun start(host: String, port: Int) {
        embeddedServer(Netty, host = host, port = port) {
            install(ContentNegotiation) {
                json()
            }
            install(StatusPages) {
                exception<VersionConflictException> { call, cause ->
                    call.respond(HttpStatusCode.Conflict, mapOf("error" to (cause.message ?: "版本冲突")))
                }
                exception<IllegalArgumentException> { call, cause ->
                    call.respond(HttpStatusCode.BadRequest, mapOf("error" to (cause.message ?: "参数错误")))
                }
                exception<Throwable> { call, cause ->
                    call.respond(
                        HttpStatusCode.InternalServerError,
                        mapOf("error" to (cause.message ?: "服务器错误"))
                    )
                }
            }
            routing {
                get("/") {
                    val html = WebServer::class.java.getResource("/web/index.html")?.readText()
                        ?: error("index.html 未找到")
                    call.respondText(html, ContentType.Text.Html)
                }

                get("/api/robots") {
                    call.respond(db.listRobots().map {
                        RobotDto(it.id, it.serial, it.name, it.importedAt.toString())
                    })
                }

                post("/api/robots") {
                    val req = call.receive<ImportRequest>()
                    val result = service.importRobot(req.serial, req.name, req.urdfXml)
                    call.respond(
                        HttpStatusCode.Created,
                        mapOf(
                            "robot" to RobotDto(
                                result.robot.id, result.robot.serial,
                                result.robot.name, result.robot.importedAt.toString()
                            ),
                            "issues" to result.issues,
                            "graph" to buildGraphDto(result.robot.urdfXml, req.serial, null),
                        )
                    )
                }

                get("/api/robots/{serial}/graph") {
                    val serial = call.parameters["serial"]!!
                    val snap = call.request.queryParameters["snapshotId"]?.toLongOrNull()
                    call.respond(buildGraphDto(robotXml(serial), serial, snap))
                }

                get("/api/robots/{serial}/urdf") {
                    val serial = call.parameters["serial"]!!
                    call.respondText(robotXml(serial), ContentType.Application.Xml)
                }

                get("/api/robots/{serial}/snapshots") {
                    val serial = call.parameters["serial"]!!
                    val json = Json { ignoreUnknownKeys = true }
                    call.respond(service.listSnapshots(serial).map { s ->
                        s.toDto(
                            json.decodeFromString<Map<String, ValueDto>>(s.jointValuesJson)
                        )
                    })
                }

                post("/api/robots/{serial}/snapshots") {
                    val serial = call.parameters["serial"]!!
                    val req = call.receive<SnapshotRequest>()
                    val id = service.createSnapshot(
                        serial, req.label, req.takenAt,
                        req.values.mapValues { JointValueInput(it.value.value, it.value.unit) }
                    )
                    call.respond(HttpStatusCode.Created, mapOf("id" to id, "frozen" to false))
                }

                post("/api/snapshots/{id}/freeze") {
                    val id = call.parameters["id"]!!.toLong()
                    service.freezeSnapshot(id)
                    call.respond(mapOf("id" to id, "frozen" to true))
                }

                get("/api/calibrations") {
                    val serial = call.request.queryParameters["serial"]
                    call.respond(service.db.listCalibrations(serial).map { it.toDto() })
                }

                post("/api/calibrations/drafts") {
                    val req = call.receive<DraftRequest>()
                    val serial = call.request.queryParameters["serial"] ?: req.robotSerial
                        ?: throw IllegalArgumentException("需要 robotSerial 字段或 ?serial= 查询参数")
                    val c = service.saveDraft(
                        req.id, serial, req.baseVersion,
                        req.parentFrame, req.childFrame, req.xyz, req.rpyRad,
                        req.note, req.validFrom, req.validTo
                    )
                    call.respond(HttpStatusCode.Created, c.toDto())
                }

                post("/api/calibrations/{id}/approve") {
                    val id = call.parameters["id"]!!
                    val req = call.receive<ApproveRequest>()
                    call.respond(service.approve(id, req.version).toDto())
                }

                post("/api/calibrations/{id}/reject") {
                    val id = call.parameters["id"]!!
                    val req = call.receive<ApproveRequest>()
                    service.reject(id, req.version)
                    call.respond(mapOf("id" to id, "version" to req.version, "status" to "REJECTED"))
                }

                post("/api/robots/{serial}/query") {
                    val serial = call.parameters["serial"]!!
                    val req = call.receive<QueryRequest>()
                    val tolRad = req.toleranceDeg?.let { Math.toRadians(it) } ?: 1e-6
                    val scenario = jcb.AppService.Scenario(
                        robotSerial = serial,
                        snapshotId = req.snapshotId,
                        from = req.from, to = req.to,
                        calibrationIds = req.calibrationIds,
                        inlineValues = req.values.mapValues {
                            JointValueInput(it.value.value, it.value.unit)
                        },
                        toleranceRad = tolRad,
                    )
                    if (req.compareCalibrationIds != null || req.compareSnapshotId != null) {
                        val other = scenario.copy(
                            calibrationIds = req.compareCalibrationIds ?: req.calibrationIds,
                            snapshotId = req.compareSnapshotId ?: req.snapshotId,
                        )
                        val cmp = service.compare(scenario, other)
                        call.respond(
                            CompareResponseDto(
                                evaluationDto(cmp.a), evaluationDto(cmp.b),
                                cmp.diffs.map {
                                    DiffEntryDto(it.parent, it.child, it.oldVersion, it.newVersion, it.changes)
                                }
                            )
                        )
                    } else {
                        call.respond(evaluationDto(service.evaluate(scenario)))
                    }
                }

                get("/api/robots/{serial}/export") {
                    val serial = call.parameters["serial"]!!
                    val ids = call.request.queryParameters.getAll("calib")
                    call.respondText(
                        service.exportDerivedUrdf(serial, ids),
                        ContentType.Application.Xml
                    )
                }
            }
        }.start(wait = true)
    }

    private fun robotXml(serial: String): String =
        db.getRobotBySerial(serial)?.urdfXml
            ?: throw IllegalArgumentException("机器人 '$serial' 不存在")

    private fun buildGraphDto(
        xml: String, serial: String, snapshotId: Long?,
    ): GraphDataDto {
        val (_, model) = UrdfParser.parse(xml)
        val at = snapshotId?.let { db.getSnapshot(it)?.takenAt }
        val approved = service.db.listCalibrations(serial)
            .filter { it.status == jcb.calib.CalibStatus.APPROVED }
        val effective = jcb.calib.CalibrationService.evaluate(approved, serial, at)
        val edges0 = jcb.calib.CalibrationService.toEdges(effective)
        val built = jcb.kin.FrameGraph.build(model, edges0)
        val linkNames = model.links.map { it.name }.toSet()
        val nodes = built.graph.frames.map { f ->
            GraphNodeDto(f, if (f in linkNames) "link" else "external-frame")
        }
        val links = built.graph.edges.map { e ->
            GraphLinkDto(
                e.parent, e.child, e.source.label,
                if (e.source.isCalibration) "calibration"
                else e.joint?.type?.urdfName ?: "joint"
            )
        }
        return GraphDataDto(
            nodes, links,
            built.graph.issues.map { IssueDto(it.kind, it.severity, it.message, it.ref) }
        )
    }
}

fun evaluationDto(r: EvaluationResult): EvaluationDto {
    val q = r.query
    return EvaluationDto(
        selectorLabel = r.selectorLabel,
        from = q.from, to = q.to,
        status = statusName(q),
        missingJoints = missingJoints(q),
        candidates = q.candidates.map { candidateDto(it) },
        spreadTranslationM = q.spread?.translationMeters,
        spreadRotationDeg = spreadDeg(q.spread),
        effectiveCalibrations = r.effective.map { eff ->
            EffectiveDto(
                eff.transform.id, eff.transform.version, eff.state.name, eff.effective,
                eff.reason, eff.transform.parentFrame, eff.transform.childFrame,
                eff.transform.validFrom?.toString(), eff.transform.validTo?.toString()
            )
        },
        cycles = r.cycles.cycles.map(::cycleDto),
        inconsistentCycles = r.cycles.inconsistentCycles.map(::cycleDto),
        minContradictingEdgeSets = r.cycles.minContradictingEdgeSets,
        cycleNote = r.cycles.note,
        issues = r.graphIssues.map { gi -> IssueDto(gi.kind, gi.severity, gi.message, gi.ref) },
    )
}
