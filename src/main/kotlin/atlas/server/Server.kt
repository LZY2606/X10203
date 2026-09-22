package atlas.server

import atlas.kinematics.CalibrationSet
import atlas.kinematics.Engine
import atlas.kinematics.QueryResult
import atlas.store.PatchOp
import atlas.store.Snapshot
import atlas.store.Store
import atlas.urdf.PatchApplier
import atlas.urdf.UrdfDiff
import atlas.urdf.UrdfParser
import io.ktor.http.ContentType
import io.ktor.http.HttpStatusCode
import io.ktor.server.application.Application
import io.ktor.server.application.ApplicationCall
import io.ktor.server.engine.EmbeddedServer
import io.ktor.server.engine.embeddedServer
import io.ktor.server.netty.Netty
import io.ktor.server.request.receiveText
import io.ktor.server.response.respond
import io.ktor.server.response.respondText
import io.ktor.server.routing.get
import io.ktor.server.routing.post
import io.ktor.server.routing.routing
import kotlinx.serialization.json.Json
import kotlinx.serialization.json.decodeFromJsonElement
import kotlinx.serialization.json.JsonObject
import kotlinx.serialization.json.JsonPrimitive
import kotlinx.serialization.json.double
import kotlinx.serialization.json.jsonObject
import kotlinx.serialization.json.jsonPrimitive
import kotlinx.serialization.json.long

object JointAtlasServer {
    private val json = Json { ignoreUnknownKeys = true; prettyPrint = false }

    fun create(store: Store, host: String, port: Int): EmbeddedServer<*, *> =
        embeddedServer(Netty, port = port, host = host) { module(store) }

    fun Application.module(store: Store) {
        routing {
            get("/") {
                call.respondText(PAGE, ContentType.Text.Html)
            }

            // ---------- URDF ----------
            post("/api/urdf") {
                val body = Json.parseToJsonElement(call.receiveText()).jsonObject
                val name = body["name"]?.jsonPrimitive?.content ?: "robot"
                val xml = body["xml"]?.jsonPrimitive?.content
                    ?: return@post call.respond(HttpStatusCode.BadRequest, "missing xml")
                try {
                    UrdfParser.parse(xml) // validate
                } catch (e: Exception) {
                    return@post call.respond(HttpStatusCode.BadRequest, "URDF parse error: ${e.message}")
                }
                val doc = store.insertUrdf(name, xml)
                call.respondText(json.encodeToString(doc), ContentType.Application.Json)
            }
            get("/api/urdf") {
                call.respondText(json.encodeToString(store.listUrdf()), ContentType.Application.Json)
            }
            get("/api/urdf/{id}") {
                val doc = store.getUrdf(call.id()) ?: return@get call.respond(HttpStatusCode.NotFound)
                call.respondText(json.encodeToString(doc), ContentType.Application.Json)
            }
            get("/api/urdf/{id}/export") {
                val doc = store.getUrdf(call.id()) ?: return@get call.respond(HttpStatusCode.NotFound)
                call.respondText(doc.xml, ContentType.Text.Xml)
            }
            get("/api/urdf/{id}/diff") {
                val id = call.id()
                val from = call.request.queryParameters["from"]?.toIntOrNull()
                val to = call.request.queryParameters["to"]?.toIntOrNull()
                if (from == null || to == null) return@get call.respond(HttpStatusCode.BadRequest, "need from&to")
                val a = store.urdfVersionXml(id, from) ?: return@get call.respond(HttpStatusCode.NotFound, "version $from not found")
                val b = store.urdfVersionXml(id, to) ?: return@get call.respond(HttpStatusCode.NotFound, "version $to not found")
                call.respondText(json.encodeToString(UrdfDiff.diff(a, b)), ContentType.Application.Json)
            }

            // ---------- Calibrations ----------
            post("/api/calibrations") {
                val set = json.decodeFromString<CalibrationSet>(call.receiveText())
                val id = store.insertCalibration(set)
                call.respondText(json.encodeToString(store.getCalibration(id)), ContentType.Application.Json)
            }
            get("/api/calibrations") {
                call.respondText(json.encodeToString(store.listCalibrations()), ContentType.Application.Json)
            }

            // ---------- Snapshots ----------
            post("/api/snapshots") {
                val body = Json.parseToJsonElement(call.receiveText()).jsonObject
                val jv = body["jointValues"]?.jsonObject?.mapValues { it.value.jsonPrimitive.double } ?: emptyMap()
                val snap = Snapshot(
                    id = 0,
                    name = body["name"]?.jsonPrimitive?.content ?: "snapshot",
                    robotSerial = body["robotSerial"]?.jsonPrimitive?.content,
                    jointValues = jv,
                    capturedAt = body["capturedAt"]?.jsonPrimitive?.long ?: System.currentTimeMillis(),
                    calibrationId = body["calibrationId"]?.jsonPrimitive?.long
                )
                val id = store.insertSnapshot(snap)
                call.respondText(json.encodeToString(store.getSnapshot(id)), ContentType.Application.Json)
            }
            get("/api/snapshots") {
                call.respondText(json.encodeToString(store.listSnapshots()), ContentType.Application.Json)
            }

            // ---------- Query / graph / cycles ----------
            get("/api/query") {
                val engine = buildEngine(store, call) ?: return@get call.respond(HttpStatusCode.BadRequest, "unknown urdfId")
                val from = call.request.queryParameters["from"]
                val to = call.request.queryParameters["to"]
                if (from == null || to == null) return@get call.respond(HttpStatusCode.BadRequest, "need from&to")
                val deg = call.request.queryParameters["units"] == "deg"
                val result = engine.query(from, to).withUnits(if (deg) "deg" else "rad")
                call.respondText(json.encodeToString(result), ContentType.Application.Json)
            }
            get("/api/graph") {
                val engine = buildEngine(store, call) ?: return@get call.respond(HttpStatusCode.BadRequest, "unknown urdfId")
                call.respondText(json.encodeToString(engine.graphView()), ContentType.Application.Json)
            }
            get("/api/cycles") {
                val engine = buildEngine(store, call) ?: return@get call.respond(HttpStatusCode.BadRequest, "unknown urdfId")
                call.respondText(json.encodeToString(engine.cycleReport()), ContentType.Application.Json)
            }

            // ---------- Drafts ----------
            post("/api/drafts") {
                val body = Json.parseToJsonElement(call.receiveText()).jsonObject
                val urdfId = body["urdfId"]?.jsonPrimitive?.long ?: return@post call.respond(HttpStatusCode.BadRequest, "need urdfId")
                val baseVersion = body["baseVersion"]?.jsonPrimitive?.long?.toInt() ?: return@post call.respond(HttpStatusCode.BadRequest, "need baseVersion")
                val ops = json.decodeFromJsonElement<List<PatchOp>>(body["ops"] ?: return@post call.respond(HttpStatusCode.BadRequest, "need ops"))
                val id = store.insertDraft(urdfId, baseVersion, ops)
                call.respondText(json.encodeToString(store.getDraft(id)), ContentType.Application.Json)
            }
            get("/api/drafts") {
                call.respondText(json.encodeToString(store.listDrafts()), ContentType.Application.Json)
            }
            post("/api/drafts/{id}/approve") {
                val id = call.id()
                store.getDraft(id) ?: return@post call.respond(HttpStatusCode.NotFound)
                store.setDraftStatus(id, "APPROVED")
                call.respondText(json.encodeToString(store.getDraft(id)), ContentType.Application.Json)
            }
            post("/api/drafts/{id}/reject") {
                val id = call.id()
                store.getDraft(id) ?: return@post call.respond(HttpStatusCode.NotFound)
                store.setDraftStatus(id, "REJECTED")
                call.respondText(json.encodeToString(store.getDraft(id)), ContentType.Application.Json)
            }
            post("/api/drafts/{id}/apply") {
                val id = call.id()
                val draft = store.getDraft(id) ?: return@post call.respond(HttpStatusCode.NotFound)
                val doc = store.getUrdf(draft.urdfId) ?: return@post call.respond(HttpStatusCode.NotFound, "urdf missing")
                val newXml = try {
                    PatchApplier.apply(UrdfParser.parse(doc.xml).document, draft.ops).serialize()
                } catch (e: Exception) {
                    return@post call.respond(HttpStatusCode.BadRequest, "patch error: ${e.message}")
                }
                val (ok, msg) = store.applyDraft(id, newXml)
                if (!ok) call.respond(HttpStatusCode.Conflict, msg)
                else call.respondText(msg)
            }

            // ---------- Snapshot compare ----------
            get("/api/compare") {
                val snapshotId = call.request.queryParameters["snapshotId"]?.toLongOrNull()
                    ?: return@get call.respond(HttpStatusCode.BadRequest, "need snapshotId")
                val otherCalId = call.request.queryParameters["calibrationId"]?.toLongOrNull()
                    ?: return@get call.respond(HttpStatusCode.BadRequest, "need calibrationId")
                val urdfId = call.request.queryParameters["urdfId"]?.toLongOrNull()
                    ?: return@get call.respond(HttpStatusCode.BadRequest, "need urdfId")
                val snap = store.getSnapshot(snapshotId) ?: return@get call.respond(HttpStatusCode.NotFound, "snapshot not found")
                val doc = store.getUrdf(urdfId) ?: return@get call.respond(HttpStatusCode.NotFound, "urdf not found")
                val robot = UrdfParser.parse(doc.xml)
                val allCals = store.listCalibrations()
                val calA = allCals.filter { it.id == snap.calibrationId }
                val calB = allCals.filter { it.id == otherCalId }
                val engA = Engine(robot, calA, snap.jointValues, snap.robotSerial, snap.capturedAt)
                val engB = Engine(robot, calB, snap.jointValues, snap.robotSerial, snap.capturedAt)
                val root = robot.links.firstOrNull()?.name ?: return@get call.respond(HttpStatusCode.BadRequest, "no links")
                val frames = (engA.frames() + engB.frames()).toSortedSet()
                val rows = frames.mapNotNull { f ->
                    if (f == root) return@mapNotNull null
                    val qa = engA.query(root, f)
                    val qb = engB.query(root, f)
                    val ma = qa.candidates.firstOrNull()?.matrix
                    val mb = qb.candidates.firstOrNull()?.matrix
                    val dev = if (ma != null && mb != null)
                        atlas.kinematics.Mat.maxDiff(ma.toDoubleArray(), mb.toDoubleArray()) else null
                    JsonObject(
                        mapOf(
                            "frame" to JsonPrimitive(f),
                            "statusA" to JsonPrimitive(qa.status),
                            "statusB" to JsonPrimitive(qb.status),
                            "deviation" to JsonPrimitive(dev),
                            "changed" to JsonPrimitive(dev != null && dev > 1e-6)
                        )
                    )
                }
                call.respondText(JsonObject(mapOf("root" to JsonPrimitive(root), "rows" to kotlinx.serialization.json.JsonArray(rows))).toString(), ContentType.Application.Json)
            }
        }
    }

    private fun ApplicationCall.id(): Long =
        parameters["id"]?.toLongOrNull() ?: -1L

    private fun buildEngine(store: Store, call: ApplicationCall): Engine? {
        val p = call.request.queryParameters
        val urdfId = p["urdfId"]?.toLongOrNull() ?: store.listUrdf().firstOrNull()?.id ?: return null
        val doc = store.getUrdf(urdfId) ?: return null
        val robot = UrdfParser.parse(doc.xml)
        val calIds = p["calibrationIds"]?.split(",")?.mapNotNull { it.toLongOrNull() }
        val cals = store.listCalibrations().filter { calIds == null || it.id in calIds }
        val snapshotId = p["snapshotId"]?.toLongOrNull()
        val snap = snapshotId?.let { store.getSnapshot(it) }
        val jointValues = snap?.jointValues
            ?: p["joints"]?.split(",")?.mapNotNull {
                val kv = it.split("=")
                if (kv.size == 2) kv[0] to (kv[1].toDoubleOrNull() ?: return@mapNotNull null) else null
            }?.toMap()
            ?: emptyMap()
        val serial = p["serial"] ?: snap?.robotSerial
        val at = p["at"]?.toLongOrNull() ?: snap?.capturedAt
        return Engine(robot, cals, jointValues, serial, at)
    }

    fun QueryResult.withUnits(units: String): QueryResult {
        if (units != "deg") return copy(units = "rad")
        return copy(
            units = "deg",
            joints = joints.map { it.copy(value = Math.toDegrees(it.value)) },
            candidates = candidates.map { c ->
                c.copy(edges = c.edges.map { e -> e.copy(jointValue = e.jointValue?.let(Math::toDegrees)) })
            }
        )
    }
}
