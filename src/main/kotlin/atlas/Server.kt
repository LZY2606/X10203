package atlas

import io.ktor.http.ContentType
import io.ktor.http.HttpStatusCode
import io.ktor.serialization.kotlinx.json.json
import io.ktor.server.application.Application
import io.ktor.server.application.install
import io.ktor.server.http.content.staticResources
import io.ktor.server.plugins.contentnegotiation.ContentNegotiation
import io.ktor.server.request.receiveText
import io.ktor.server.response.respond
import io.ktor.server.response.respondText
import io.ktor.server.routing.get
import io.ktor.server.routing.post
import io.ktor.server.routing.routing
import kotlinx.serialization.json.JsonNull
import kotlinx.serialization.json.JsonArray
import kotlinx.serialization.json.JsonObject
import kotlinx.serialization.json.JsonPrimitive
import kotlinx.serialization.json.buildJsonArray
import kotlinx.serialization.json.buildJsonObject

fun Mat4.toJson(): JsonArray = buildJsonArray {
    rows().forEach { r -> add(JsonArray(r.map { JsonPrimitive(it) })) }
}

fun parseJointValues(s: String?): Map<String, Double> {
    if (s.isNullOrBlank()) return emptyMap()
    return s.split(",").mapNotNull { part ->
        val kv = part.split("=", ":")
        if (kv.size >= 2) {
            val v = kv[1].toDoubleOrNull()
            if (v != null) kv[0].trim() to v else null
        } else null
    }.toMap()
}

fun QueryResult.toJson(): JsonObject = buildJsonObject {
    put("from", JsonPrimitive(from))
    put("to", JsonPrimitive(to))
    put("diagnostics", JsonArray(diagnostics.map { JsonPrimitive(it) }))
    put("pathError", pathError?.let { JsonPrimitive(it) } ?: JsonNull)
    put("candidates", JsonArray(candidates.map { c ->
        buildJsonObject {
            put("frames", JsonArray(c.frames.map { JsonPrimitive(it) }))
            put("sources", JsonArray(c.sources.map { JsonPrimitive(it) }))
            put("joints", JsonArray(c.joints.map { j ->
                buildJsonObject {
                    put("joint", JsonPrimitive(j.joint))
                    put("value", JsonPrimitive(j.value))
                    put("unit", JsonPrimitive(j.unit))
                    put("status", JsonPrimitive(j.status))
                }
            }))
            put("matrix", c.matrix.toJson())
        }
    }))
}

fun Application.module(atlas: Atlas) {
    install(ContentNegotiation) { json() }
    routing {
        staticResources("/", "static", index = "index.html")

        get("/api/state") {
            val versions = atlas.store.listVersions()
            call.respond(buildJsonObject {
                put("current", JsonPrimitive(atlas.store.currentVersion()))
                put("versions", JsonArray(versions.map {
                    buildJsonObject {
                        put("id", JsonPrimitive(it.id))
                        put("name", JsonPrimitive(it.name))
                        put("createdAt", JsonPrimitive(it.createdAt))
                    }
                }))
                put("calibs", JsonArray(atlas.store.calibs().map { c ->
                    buildJsonObject {
                        put("id", JsonPrimitive(c.id))
                        put("version", JsonPrimitive(c.version))
                        put("from", JsonPrimitive(c.from))
                        put("to", JsonPrimitive(c.to))
                        put("xyz", JsonPrimitive(c.xyz.joinToString(" ")))
                        put("rpy", JsonPrimitive(c.rpy.joinToString(" ")))
                        put("unit", JsonPrimitive(c.rpyUnit))
                        put("serial", JsonPrimitive(c.serial))
                        put("validFrom", JsonPrimitive(c.validFrom))
                        put("validTo", JsonPrimitive(c.validTo))
                    }
                }))
                put("drafts", JsonArray(atlas.store.listDrafts().map { d ->
                    buildJsonObject {
                        put("id", JsonPrimitive(d.id))
                        put("baseVersion", JsonPrimitive(d.baseVersion))
                        put("description", JsonPrimitive(d.description))
                        put("patch", JsonPrimitive(d.patch))
                        put("status", JsonPrimitive(d.status))
                    }
                }))
                put("snapshots", JsonArray(atlas.store.snapshots().map { s ->
                    buildJsonObject {
                        put("id", JsonPrimitive(s.id))
                        put("name", JsonPrimitive(s.name))
                        put("joints", JsonPrimitive(s.joints))
                        put("note", JsonPrimitive(s.note))
                        put("createdAt", JsonPrimitive(s.createdAt))
                    }
                }))
            })
        }

        post("/api/urdf") {
            val name = call.request.queryParameters["name"] ?: "robot"
            val xml = call.receiveText()
            try {
                val r = atlas.importUrdf(name, xml)
                call.respond(buildJsonObject {
                    put("version", JsonPrimitive(r.version))
                    put("diagnostics", JsonArray(r.diagnostics.map { JsonPrimitive(it) }))
                })
            } catch (e: Exception) {
                call.respond(HttpStatusCode.BadRequest, buildJsonObject {
                    put("error", JsonPrimitive("URDF 解析失败: ${e.message}"))
                })
            }
        }

        get("/api/export") {
            val v = call.request.queryParameters["version"]?.toIntOrNull()
            val uv = atlas.store.getUrdf(v)
            if (uv == null) {
                call.respond(HttpStatusCode.NotFound, buildJsonObject { put("error", JsonPrimitive("无 URDF 版本")) })
            } else {
                call.respondText(uv.xml, ContentType.Text.Xml)
            }
        }

        post("/api/calib") {
            val p = call.request.queryParameters
            fun req(k: String) = p[k] ?: throw IllegalArgumentException("缺少参数 $k")
            try {
                val c = atlas.addCalib(
                    req("id"), req("from"), req("to"),
                    parseDoubles(p["xyz"], doubleArrayOf(0.0, 0.0, 0.0)),
                    parseDoubles(p["rpy"], doubleArrayOf(0.0, 0.0, 0.0)),
                    p["unit"] ?: "rad",
                    req("serial"),
                    req("validFrom").toLong(),
                    req("validTo").toLong()
                )
                call.respond(buildJsonObject {
                    put("id", JsonPrimitive(c.id))
                    put("version", JsonPrimitive(c.version))
                })
            } catch (e: Exception) {
                call.respond(HttpStatusCode.BadRequest, buildJsonObject {
                    put("error", JsonPrimitive(e.message ?: "bad request"))
                })
            }
        }

        get("/api/query") {
            val p = call.request.queryParameters
            val from = p["from"]; val to = p["to"]
            if (from == null || to == null) {
                call.respond(HttpStatusCode.BadRequest, buildJsonObject { put("error", JsonPrimitive("缺少 from/to")) })
                return@get
            }
            val engine = atlas.engine(p["serial"] ?: "", p["time"]?.toLongOrNull() ?: 0L)
            if (engine == null) {
                call.respond(HttpStatusCode.BadRequest, buildJsonObject { put("error", JsonPrimitive("尚未导入 URDF")) })
                return@get
            }
            val r = engine.query(from, to, parseJointValues(p["joints"]))
            val json = r.toJson()
            val all = buildJsonObject {
                json.forEach { (k, v) -> put(k, v) }
                put("engineDiagnostics", JsonArray(engine.diagnostics.map { JsonPrimitive(it) }))
            }
            call.respond(all)
        }

        get("/api/cycles") {
            val p = call.request.queryParameters
            val engine = atlas.engine(p["serial"] ?: "", p["time"]?.toLongOrNull() ?: 0L)
            if (engine == null) {
                call.respond(HttpStatusCode.BadRequest, buildJsonObject { put("error", JsonPrimitive("尚未导入 URDF")) })
                return@get
            }
            val a = engine.cycles(parseJointValues(p["joints"]))
            call.respond(buildJsonObject {
                put("cycles", JsonArray(a.cycles.map { c ->
                    buildJsonObject {
                        put("frames", JsonArray(c.frames.map { JsonPrimitive(it) }))
                        put("edges", JsonArray(c.edgeIds.map { JsonPrimitive(it) }))
                        put("residual", JsonPrimitive(c.residual))
                        put("consistent", JsonPrimitive(c.consistent))
                    }
                }))
                put("minContradictionEdges", JsonArray(a.minContradictionEdges.map { JsonPrimitive(it) }))
                put("diagnostics", JsonArray(engine.diagnostics.map { JsonPrimitive(it) }))
            })
        }

        get("/api/graph") {
            val p = call.request.queryParameters
            val engine = atlas.engine(p["serial"] ?: "", p["time"]?.toLongOrNull() ?: 0L)
            if (engine == null) {
                call.respond(HttpStatusCode.BadRequest, buildJsonObject { put("error", JsonPrimitive("尚未导入 URDF")) })
                return@get
            }
            call.respond(buildJsonObject {
                put("nodes", JsonArray(engine.edges.flatMap { listOf(it.from, it.to) }.distinct().map { JsonPrimitive(it) }))
                put("edges", JsonArray(engine.edges.map { e ->
                    buildJsonObject {
                        put("id", JsonPrimitive(e.id))
                        put("from", JsonPrimitive(e.from))
                        put("to", JsonPrimitive(e.to))
                        put("kind", JsonPrimitive(e.kind.name))
                        put("source", JsonPrimitive(e.source))
                    }
                }))
                put("diagnostics", JsonArray(engine.diagnostics.map { JsonPrimitive(it) }))
            })
        }

        post("/api/drafts") {
            val p = call.request.queryParameters
            val base = p["baseVersion"]?.toIntOrNull()
            if (base == null) {
                call.respond(HttpStatusCode.BadRequest, buildJsonObject { put("error", JsonPrimitive("缺少 baseVersion")) })
                return@post
            }
            val id = atlas.store.createDraft(base, p["description"] ?: "", call.receiveText())
            call.respond(buildJsonObject { put("id", JsonPrimitive(id)) })
        }

        post("/api/drafts/{id}/approve") {
            val id = call.parameters["id"]?.toIntOrNull()
            if (id == null) {
                call.respond(HttpStatusCode.BadRequest)
                return@post
            }
            when (atlas.approveDraft(id)) {
                ApproveResult.OK -> call.respond(buildJsonObject { put("status", JsonPrimitive("APPROVED")) })
                ApproveResult.CONFLICT -> call.respond(HttpStatusCode.Conflict, buildJsonObject {
                    put("error", JsonPrimitive("版本冲突: baseVersion 与当前版本不一致"))
                })
                ApproveResult.NOT_PENDING -> call.respond(HttpStatusCode.Conflict, buildJsonObject {
                    put("error", JsonPrimitive("草案不是待审批状态"))
                })
                ApproveResult.BAD_PATCH -> call.respond(HttpStatusCode.BadRequest, buildJsonObject {
                    put("error", JsonPrimitive("补丁无法应用"))
                })
                ApproveResult.NOT_FOUND -> call.respond(HttpStatusCode.NotFound, buildJsonObject {
                    put("error", JsonPrimitive("草案不存在"))
                })
            }
        }

        post("/api/drafts/{id}/reject") {
            val id = call.parameters["id"]?.toIntOrNull()
            if (id != null && atlas.rejectDraft(id)) {
                call.respond(buildJsonObject { put("status", JsonPrimitive("REJECTED")) })
            } else {
                call.respond(HttpStatusCode.Conflict, buildJsonObject { put("error", JsonPrimitive("无法驳回")) })
            }
        }

        post("/api/snapshots") {
            val p = call.request.queryParameters
            val id = atlas.store.addSnapshot(p["name"] ?: "snapshot", p["joints"] ?: "", p["note"] ?: "")
            call.respond(buildJsonObject { put("id", JsonPrimitive(id)) })
        }

        get("/api/diff") {
            val v1 = call.request.queryParameters["v1"]?.toIntOrNull()
            val v2 = call.request.queryParameters["v2"]?.toIntOrNull()
            val d = if (v1 != null && v2 != null) atlas.diff(v1, v2) else null
            if (d == null) {
                call.respond(HttpStatusCode.NotFound, buildJsonObject { put("error", JsonPrimitive("版本不存在")) })
            } else {
                call.respond(buildJsonObject { put("lines", JsonArray(d.map { JsonPrimitive(it) })) })
            }
        }
    }
}
