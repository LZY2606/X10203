package coordbook

import io.ktor.http.ContentType
import io.ktor.http.HttpStatusCode
import io.ktor.server.application.call
import io.ktor.server.request.receive
import io.ktor.server.response.respond
import io.ktor.server.response.respondText
import io.ktor.server.routing.Route
import io.ktor.server.routing.get
import io.ktor.server.routing.post
import io.ktor.server.routing.route
import kotlin.math.PI

fun Route.apiRoutes(service: RobotService, db: Database) {

    fun requireRobot(id: Long): Robot =
        db.getRobot(id) ?: throw NoSuchElementException("机器人 #$id 不存在")

    route("/api/robots") {
        get {
            val list = db.listRobots().map { r ->
                val doc = UrdfParser.parse(r.urdfXml)
                RobotSummaryDto(
                    r.id, r.name, r.serial, r.currentRevision,
                    doc.links.size, doc.joints.size,
                )
            }
            call.respond(list)
        }
        post {
            val req = call.receive<ImportRobotRequest>()
            UrdfParser.parse(req.urdf) // 先校验
            val robot = db.insertRobot(req.name, req.serial, req.urdf)
            call.respond(HttpStatusCode.Created, robotSummary(robot))
        }

        route("/{id}") {
            get {
                val robot = requireRobot(call.parameters["id"]!!.toLong())
                val doc = UrdfParser.parse(robot.urdfXml)
                val (graph, graphIssues) = GraphBuilder.build(doc)
                val components = GraphStats.disconnectedComponents(graph)
                call.respond(
                    RobotDetailDto(
                        robot.id, robot.name, robot.serial, robot.currentRevision,
                        doc.links.values.sortedBy { it.ordinal }.map(DtoConversions::link),
                        doc.joints.values.sortedBy { it.ordinal }.map(DtoConversions::joint),
                        doc.issues.map(DtoConversions::issue),
                        graphIssues.map(DtoConversions::issue),
                        components,
                    ),
                )
            }

            get("/raw") {
                val robot = requireRobot(call.parameters["id"]!!.toLong())
                call.respondText(XmlPatcher.exportRaw(UrdfParser.parse(robot.urdfXml)), ContentType.Application.Xml)
            }

            post("/urdf") {
                val id = call.parameters["id"]!!.toLong()
                val req = call.receive<UpdateUrdfRequest>()
                UrdfParser.parse(req.urdf)
                val updated = db.updateUrdf(id, req.urdf, req.expectedRevision)
                call.respond(robotSummary(updated))
            }

            get("/snapshots") {
                val robot = requireRobot(call.parameters["id"]!!.toLong())
                call.respond(db.listSnapshots(robot.id).map(DtoConversions::snapshot))
            }
            post("/snapshots") {
                val id = call.parameters["id"]!!.toLong()
                val robot = requireRobot(id)
                val req = call.receive<CreateSnapshotRequest>()
                IsoTime.parse(req.frozenAt)
                val snap = db.insertSnapshot(
                    robot.id, req.serial, req.frozenAt,
                    JsonMaps.encodeDoubles(req.jointValues), req.note,
                )
                call.respond(HttpStatusCode.Created, DtoConversions.snapshot(snap))
            }

            get("/calibrations") {
                val robot = requireRobot(call.parameters["id"]!!.toLong())
                val versions = db.listCalibVersions(robot.id)
                val patches = db.listPatches()
                call.respond(versions.map { DtoConversions.version(it, patches) })
            }

            get("/cycles") {
                val robot = requireRobot(call.parameters["id"]!!.toLong())
                val at = call.request.queryParameters["at"] ?: IsoTime.now()
                val calib = call.request.queryParameters["calibVersionId"]?.toLong()
                val report = service.cycleReport(robot, at, calib)
                call.respond(
                    CycleResponseDto(
                        report.hasCycle, report.inconsistent, report.note,
                        report.residuals.map {
                            CycleResidualDto(
                                it.frames, it.edgeLabels, it.closureTransform,
                                it.translationErrorM, it.rotationErrorRad, it.consistent,
                            )
                        },
                        report.minimalContradictingEdges, report.spanningTreeEdgeIds,
                    ),
                )
            }

            post("/query") {
                val id = call.parameters["id"]!!.toLong()
                val robot = requireRobot(id)
                val req = call.receive<QueryRequest>()
                IsoTime.parse(req.at)
                val snapshot = req.snapshotId?.let {
                    db.getSnapshot(it) ?: throw NoSuchElementException("快照 #$it 不存在")
                }
                // 页面角度以度输入；rad 覆盖优先，随后把 degree 值作用到 revolute 关节
                val values = HashMap<String, Double>()
                if (snapshot != null) values.putAll(snapshot.jointValues)
                req.jointValuesDeg?.forEach { (name, deg) -> values[name] = deg * PI / 180.0 }
                req.jointValuesRad?.forEach { (name, rad) -> values[name] = rad }
                val doc = UrdfParser.parse(robot.urdfXml)
                val effective = service.graphFor(robot, snapshot, req.at, req.calibVersionId)
                val result = Kinematics.query(effective.graph, doc, req.from, req.to, values)
                call.respond(toQueryDto(req, result, effective))
            }

            get("/export") {
                val robot = requireRobot(call.parameters["id"]!!.toLong())
                val calib = call.request.queryParameters["calibVersionId"]?.toLong()
                val doc = UrdfParser.parse(robot.urdfXml)
                val patches = if (calib != null) db.listPatches(calib) else db.listPatches()
                val (xml, applied) = XmlPatcher.applyApprovedPatches(doc, patches)
                call.respond(
                    ExportResponseDto(
                        xml,
                        applied.map { AppliedPatchDto(it.patchId, it.jointName, it.changed) },
                        "导出文件仅应用状态为 APPROVED 的补丁；标定额外变换独立版本化，不写入本 URDF",
                    ),
                )
            }

            get("/export/calibration") {
                val robot = requireRobot(call.parameters["id"]!!.toLong())
                val calibId = call.request.queryParameters["calibVersionId"]?.toLong()
                    ?: throw IllegalArgumentException("需要 calibVersionId 参数")
                val version = db.getCalibVersion(calibId)
                    ?: throw NoSuchElementException("标定版本 #$calibId 不存在")
                call.respondText(
                    CalibExporter.export(version, db.listPatches(calibId)),
                    ContentType.Application.Xml,
                )
            }
        }
    }

    route("/api/calibrations/versions") {
        post {
            val req = call.receive<CreateVersionRequest>()
            IsoTime.parse(req.validFrom); IsoTime.parse(req.validUntil)
            val robotId = call.request.queryParameters["robotId"]?.toLong()
                ?: throw IllegalArgumentException("需要 robotId 查询参数")
            requireRobot(robotId)
            val v = db.insertCalibVersion(
                robotId, req.serialScope, req.validFrom, req.validUntil, req.note,
            )
            call.respond(HttpStatusCode.Created, DtoConversions.version(v, emptyList()))
        }
    }

    route("/api/calibrations/patches") {
        post {
            val req = call.receive<CreatePatchRequest>()
            db.getCalibVersion(req.versionId)
                ?: throw NoSuchElementException("标定版本 #${req.versionId} 不存在")
            val kind = PatchKind.entries.firstOrNull { it.name == req.kind }
                ?: throw IllegalArgumentException("未知补丁类型 ${req.kind}")
            val patch = db.insertPatch(
                req.versionId, kind, req.targetJoint, req.parentFrame, req.childFrame,
                req.xyz?.toDoubleArray(), req.rpyRadians?.toDoubleArray(), req.note,
            )
            call.respond(HttpStatusCode.Created, DtoConversions.patch(patch))
        }

        post("/{id}/status") {
            val id = call.parameters["id"]!!.toLong()
            val req = call.receive<ApproveRequest>()
            val status = PatchStatus.entries.firstOrNull { it.name == req.status }
                ?: throw IllegalArgumentException("状态应为 PENDING/APPROVED/REJECTED")
            val updated = db.setPatchStatus(id, status, req.expectedRevision)
            call.respond(DtoConversions.patch(updated))
        }
    }

    route("/api/calibrations/diff") {
        post {
            val req = call.receive<DiffRequest>()
            val base = db.getCalibVersion(req.baseVersionId)
                ?: throw NoSuchElementException("基线版本 #${req.baseVersionId} 不存在")
            val target = db.getCalibVersion(req.targetVersionId)
                ?: throw NoSuchElementException("对比版本 #${req.targetVersionId} 不存在")
            val d = VersionDiffer.diff(
                db.listPatches(base.id), db.listPatches(target.id),
                base.revision, target.revision,
            )
            call.respond(
                VersionDiffDto(
                    d.baseRevision, d.targetRevision,
                    d.added.map(DtoConversions::patch),
                    d.removed.map(DtoConversions::patch),
                    d.changedStatus.map {
                        StatusChangeDto(it.patchId, it.from.name, it.to.name, it.kind.name)
                    },
                ),
            )
        }
    }
}

private fun robotSummary(robot: Robot): RobotSummaryDto {
    val doc = UrdfParser.parse(robot.urdfXml)
    return RobotSummaryDto(
        robot.id, robot.name, robot.serial, robot.currentRevision,
        doc.links.size, doc.joints.size,
    )
}

private fun toQueryDto(
    req: QueryRequest,
    result: FrameQueryResult,
    effective: EffectiveGraph,
): QueryResponseDto {
    val report = result.report
    val effectiveDto = EffectiveDto(
        effective.effective.state.name,
        effective.effective.reason,
        effective.calibVersion?.id,
        effective.calibVersion?.revision,
        effective.appliedPatchIds,
    )
    val candidates = report?.candidates?.map { c ->
        PathCandidateDto(
            c.index, c.frames,
            c.steps.map { s ->
                val radOrM = s.jointValue
                val deg = if (s.valueUnit == "rad" && radOrM != null) radOrM * 180.0 / PI else null
                ChainStepDto(
                    s.fromFrame, s.toFrame, s.edgeLabel, s.edgeKind,
                    s.jointName, s.jointType, radOrM, deg, s.valueUnit, s.valueOrigin,
                    s.transform,
                )
            },
            c.transform,
        )
    } ?: emptyList()
    return QueryResponseDto(
        ok = result.ok,
        error = result.error,
        ambiguousFrameIds = result.ambiguity,
        fromFrame = req.from,
        toFrame = req.to,
        effective = effectiveDto,
        candidates = candidates,
        candidateCount = candidates.size,
        consistent = report?.consistent ?: true,
        maxTranslationErrorM = report?.maxTranslationErrorM ?: 0.0,
        maxRotationErrorRad = report?.maxRotationErrorRad ?: 0.0,
        consensusTransform = report?.consensus,
        missingJoint = report?.missingJoint?.let { MissingJointDto(it.jointName, it.reason) },
    )
}
