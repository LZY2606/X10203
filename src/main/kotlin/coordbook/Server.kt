package coordbook

import io.ktor.http.ContentType
import io.ktor.http.HttpStatusCode
import io.ktor.serialization.kotlinx.json.json
import io.ktor.server.application.Application
import io.ktor.server.application.install
import io.ktor.server.engine.EmbeddedServer
import io.ktor.server.engine.embeddedServer
import io.ktor.server.netty.Netty
import io.ktor.server.plugins.contentnegotiation.ContentNegotiation
import io.ktor.server.request.receiveText
import io.ktor.server.response.respond
import io.ktor.server.response.respondText
import io.ktor.server.routing.get
import io.ktor.server.routing.post
import io.ktor.server.routing.routing
import kotlinx.serialization.json.Json
import kotlinx.serialization.json.JsonArray
import kotlinx.serialization.json.JsonObject
import kotlinx.serialization.json.JsonPrimitive
import kotlinx.serialization.json.buildJsonArray
import kotlinx.serialization.json.buildJsonObject
import kotlinx.serialization.json.double
import kotlinx.serialization.json.jsonArray
import kotlinx.serialization.json.jsonObject
import kotlinx.serialization.json.jsonPrimitive
import kotlinx.serialization.json.long
import kotlinx.serialization.json.put
import kotlinx.serialization.json.putJsonArray
import kotlinx.serialization.json.putJsonObject

fun jp(s: String?) = if (s == null) JsonPrimitive(null as String?) else JsonPrimitive(s)

fun edgeJson(e: Edge) = buildJsonObject {
    put("id", e.id); put("from", e.from); put("to", e.to)
    put("kind", e.kind); put("source", e.source)
    put("jointName", jp(e.jointName)); put("jointType", jp(e.jointType))
    put("value", e.value?.let { JsonPrimitive(it) } ?: JsonPrimitive(null as Double?))
    put("valueUnit", jp(e.valueUnit)); put("state", e.state); put("duplicate", e.duplicate)
}

fun matrixJson(t: Transform) = buildJsonArray {
    for (row in t.rows()) add(JsonArray(row.map { JsonPrimitive(it) }))
}

class Ctx(val db: Db, val doc: Db.UrdfDocRow, val robot: UrdfRobot, val edges: List<Edge>, val time: Long?)

fun buildCtx(db: Db, serial: String, time: Long?, jointValues: Map<String, Double>?): Ctx? {
    val doc = db.latestUrdf(serial) ?: return null
    val robot = UrdfParser.parse(doc.xml)
    val jv = jointValues ?: robot.joints.mapNotNull { j ->
        when (j.type) {
            "fixed" -> null
            else -> j.name to 0.0
        }
    }.toMap()
    val edges = Kinematics.buildEdges(GraphContext(robot, serial, jv, db.calibrations(serial), time))
    return Ctx(db, doc, robot, edges, time)
}

fun Application.coordbookModule(db: Db) {
    install(ContentNegotiation) { json(Json { prettyPrint = false }) }

    fun queryTime(call: io.ktor.server.application.ApplicationCall): Long? =
        call.request.queryParameters["time"]?.toLongOrNull()

    routing {
        get("/") { call.respondText(Page.HTML, ContentType.Text.Html) }

        get("/api/state") {
            val serial = call.request.queryParameters["serial"] ?: Seed.DEMO_SERIAL
            val ctx = buildCtx(db, serial, queryTime(call), null)
                ?: return@get call.respond(HttpStatusCode.NotFound, mapOf("error" to "no urdf for serial"))
            val cycles = Kinematics.cycleResiduals(ctx.edges)
            val contradiction = Kinematics.minimalContradictionEdgeSet(cycles)
            call.respond(buildJsonObject {
                put("robot", ctx.robot.name)
                put("serial", serial)
                put("docVersion", ctx.doc.version)
                put("time", call.request.queryParameters["time"]?.toLongOrNull()?.let { JsonPrimitive(it) } ?: JsonPrimitive(null as Long?))
                putJsonArray("links") { ctx.robot.links.forEach { add(JsonPrimitive(it.name)) } }
                putJsonArray("joints") {
                    ctx.robot.joints.forEach { j ->
                        add(buildJsonObject {
                            put("name", j.name); put("type", j.type); put("parent", j.parent); put("child", j.child)
                            put("mimic", jp(j.mimic?.joint))
                        })
                    }
                }
                putJsonArray("edges") { ctx.edges.forEach { add(edgeJson(it)) } }
                putJsonArray("cycles") {
                    cycles.forEach { c ->
                        add(buildJsonObject {
                            putJsonArray("edges") { c.edges.forEach { add(JsonPrimitive(it)) } }
                            put("closingEdge", c.closingEdge)
                            put("translationError", c.translationError)
                            put("rotationErrorDeg", c.rotationError * 180.0 / Math.PI)
                            put("consistent", c.consistent)
                            put("state", c.state)
                        })
                    }
                }
                putJsonArray("contradictionEdgeSet") { contradiction.forEach { add(JsonPrimitive(it)) } }
                putJsonArray("calibrations") {
                    db.calibrations(serial).forEach { c ->
                        add(buildJsonObject {
                            put("id", c.id); put("name", c.name); put("version", c.version)
                            put("parent", c.parentFrame); put("child", c.childFrame)
                            put("angleUnit", c.angleUnit)
                            put("validFrom", c.validFrom?.let { JsonPrimitive(it) } ?: JsonPrimitive(null as Long?))
                            put("validTo", c.validTo?.let { JsonPrimitive(it) } ?: JsonPrimitive(null as Long?))
                            put("status", c.status)
                            put("activity", c.activeAt(ctx.time))
                        })
                    }
                }
                putJsonArray("urdfVersions") {
                    db.urdfVersions(serial).forEach { (v, id) ->
                        add(buildJsonObject { put("version", v); put("docId", id) })
                    }
                }
                putJsonArray("snapshots") {
                    db.snapshots().filter { it.serial == serial }.forEach { s ->
                        add(buildJsonObject {
                            put("id", s.id); put("name", s.name)
                            put("time", s.time?.let { JsonPrimitive(it) } ?: JsonPrimitive(null as Long?))
                            putJsonObject("jointValues") { s.jointValues.forEach { (k, v) -> put(k, v) } }
                        })
                    }
                }
                putJsonArray("drafts") {
                    db.drafts().forEach { d ->
                        add(buildJsonObject {
                            put("id", d.id); put("docId", d.docId); put("baseVersion", d.baseVersion)
                            put("status", d.status); put("patch", d.patchJson)
                        })
                    }
                }
            })
        }

        get("/api/query") {
            val serial = call.request.queryParameters["serial"] ?: Seed.DEMO_SERIAL
            val from = call.request.queryParameters["from"]
            val to = call.request.queryParameters["to"]
            if (from == null || to == null) return@get call.respond(HttpStatusCode.BadRequest, mapOf("error" to "from/to required"))
            val snapId = call.request.queryParameters["snapshot"]?.toLongOrNull()
            val snap = snapId?.let { db.snapshot(it) }
            val time = call.request.queryParameters["time"]?.toLongOrNull() ?: snap?.time
            val ctx = buildCtx(db, serial, time, snap?.jointValues)
                ?: return@get call.respond(HttpStatusCode.NotFound, mapOf("error" to "no urdf for serial"))
            val candidates = Kinematics.findPaths(ctx.edges, from, to)
            call.respond(buildJsonObject {
                put("from", from); put("to", to)
                put("time", time?.let { JsonPrimitive(it) } ?: JsonPrimitive(null as Long?))
                put("candidateCount", candidates.size)
                putJsonArray("candidates") {
                    candidates.forEach { c ->
                        add(buildJsonObject {
                            put("state", c.state)
                            put("errorTranslation", c.errorTranslation)
                            put("errorRotationDeg", c.errorRotation * 180.0 / Math.PI)
                            putJsonArray("chain") {
                                c.steps.forEach { s ->
                                    add(buildJsonObject {
                                        put("edge", s.edge.id); put("forward", s.forward)
                                        put("from", if (s.forward) s.edge.from else s.edge.to)
                                        put("to", if (s.forward) s.edge.to else s.edge.from)
                                        put("source", s.edge.source)
                                        put("joint", jp(s.edge.jointName))
                                        put("value", s.edge.value?.let { JsonPrimitive(it) } ?: JsonPrimitive(null as Double?))
                                        put("unit", jp(s.edge.valueUnit))
                                        put("state", s.edge.state)
                                    })
                                }
                            }
                            if (c.transform != null) put("matrix", matrixJson(c.transform))
                        })
                    }
                }
            })
        }

        post("/api/import") {
            val body = Json.parseToJsonElement(call.receiveText()).jsonObject
            val serial = body["serial"]!!.jsonPrimitive.content
            val xml = body["xml"]!!.jsonPrimitive.content
            val robot = try { UrdfParser.parse(xml) } catch (e: Exception) {
                return@post call.respond(HttpStatusCode.BadRequest, mapOf("error" to "parse: ${e.message}"))
            }
            val next = (db.latestUrdf(serial)?.version ?: 0) + 1
            val id = db.insertUrdf(robot.name, serial, next, xml)
            call.respond(mapOf("docId" to id.toString(), "version" to next.toString()))
        }

        post("/api/calibration") {
            val b = Json.parseToJsonElement(call.receiveText()).jsonObject
            fun arr(name: String) = b[name]!!.jsonArray.let { a -> doubleArrayOf(a[0].jsonPrimitive.double, a[1].jsonPrimitive.double, a[2].jsonPrimitive.double) }
            val serial = b["serial"]!!.jsonPrimitive.content
            val name = b["name"]!!.jsonPrimitive.content
            val nextVer = (db.calibrations(serial).filter { it.name == name }.maxOfOrNull { it.version } ?: 0) + 1
            val id = db.insertCalibration(
                Calibration(
                    0, serial, name,
                    b["parent"]!!.jsonPrimitive.content, b["child"]!!.jsonPrimitive.content,
                    arr("xyz"), arr("rpy"),
                    b["angleUnit"]?.jsonPrimitive?.content ?: "rad",
                    b["validFrom"]?.jsonPrimitive?.long, b["validTo"]?.jsonPrimitive?.long,
                    nextVer,
                )
            )
            call.respond(mapOf("id" to id.toString(), "version" to nextVer.toString()))
        }

        post("/api/draft") {
            val b = Json.parseToJsonElement(call.receiveText()).jsonObject
            val serial = b["serial"]!!.jsonPrimitive.content
            val doc = db.latestUrdf(serial) ?: return@post call.respond(HttpStatusCode.NotFound, mapOf("error" to "no doc"))
            val base = b["baseVersion"]?.jsonPrimitive?.content?.toInt() ?: doc.version
            if (base != doc.version) return@post call.respond(HttpStatusCode.Conflict, mapOf("error" to "stale base version"))
            val id = db.insertDraft(doc.id, base, b["patch"]!!.jsonObject.toString())
            call.respond(mapOf("id" to id.toString()))
        }

        post("/api/draft/{id}/approve") {
            val id = call.parameters["id"]!!.toLong()
            when (val r = db.approveDraft(id)) {
                "approved" -> call.respond(mapOf("status" to r))
                "conflict" -> call.respond(HttpStatusCode.Conflict, mapOf("status" to r))
                else -> call.respond(HttpStatusCode.NotFound, mapOf("status" to r))
            }
        }

        post("/api/draft/{id}/reject") {
            val id = call.parameters["id"]!!.toLong()
            call.respond(mapOf("status" to db.rejectDraft(id)))
        }

        post("/api/snapshot") {
            val b = Json.parseToJsonElement(call.receiveText()).jsonObject
            val jv = b["jointValues"]!!.jsonObject.mapValues { it.value.jsonPrimitive.double }
            val id = db.insertSnapshot(
                b["name"]!!.jsonPrimitive.content,
                b["serial"]!!.jsonPrimitive.content,
                jv, b["time"]?.jsonPrimitive?.long,
            )
            call.respond(mapOf("id" to id.toString()))
        }

        get("/api/export") {
            val serial = call.request.queryParameters["serial"] ?: Seed.DEMO_SERIAL
            val doc = db.latestUrdf(serial) ?: return@get call.respond(HttpStatusCode.NotFound, "no doc")
            val parsed = UrdfParser.parse(doc.xml)
            val approved = db.drafts().filter { it.docId == doc.id && it.status == "approved" }.map { it.patchJson }
            Patches.applyAll(parsed.document, approved)
            call.respondText(UrdfParser.serialize(parsed.document), ContentType.Text.Xml)
        }

        get("/api/diff") {
            val serial = call.request.queryParameters["serial"] ?: Seed.DEMO_SERIAL
            val v1 = call.request.queryParameters["v1"]!!.toInt()
            val v2 = call.request.queryParameters["v2"]!!.toInt()
            val ids = db.urdfVersions(serial).toMap()
            val d1 = ids[v1]?.let { db.urdfById(it) }; val d2 = ids[v2]?.let { db.urdfById(it) }
            if (d1 == null || d2 == null) return@get call.respond(HttpStatusCode.NotFound, mapOf("error" to "version missing"))
            val changes = Patches.diff(UrdfParser.parse(d1.xml), UrdfParser.parse(d2.xml))
            call.respond(buildJsonObject {
                put("v1", v1); put("v2", v2)
                putJsonArray("changes") { changes.forEach { add(JsonPrimitive(it)) } }
            })
        }

        get("/api/compare") {
            // Freeze one field snapshot, compare against another calibration context (time).
            val snapId = call.request.queryParameters["snapshot"]!!.toLong()
            val snap = db.snapshot(snapId) ?: return@get call.respond(HttpStatusCode.NotFound, mapOf("error" to "no snapshot"))
            val altTime = call.request.queryParameters["time"]?.toLongOrNull()
            val from = call.request.queryParameters["from"] ?: return@get call.respond(HttpStatusCode.BadRequest, mapOf("error" to "from required"))
            val to = call.request.queryParameters["to"] ?: return@get call.respond(HttpStatusCode.BadRequest, mapOf("error" to "to required"))
            fun poseAt(time: Long?): JsonObject {
                val ctx = buildCtx(db, snap.serial, time, snap.jointValues)!!
                val best = Kinematics.findPaths(ctx.edges, from, to).filter { it.transform != null }
                    .minByOrNull { it.errorTranslation + it.errorRotation }
                val cycles = Kinematics.cycleResiduals(ctx.edges)
                return buildJsonObject {
                    put("time", time?.let { JsonPrimitive(it) } ?: JsonPrimitive(null as Long?))
                    if (best?.transform != null) put("matrix", matrixJson(best.transform))
                    put("inconsistentCycles", cycles.count { !it.consistent })
                }
            }
            call.respond(buildJsonObject {
                put("snapshot", snap.name)
                put("frozen", poseAt(snap.time))
                put("alternate", poseAt(altTime))
            })
        }
    }
}

fun createServer(db: Db, host: String, port: Int): EmbeddedServer<*, *> =
    embeddedServer(Netty, port = port, host = host) { coordbookModule(db) }
