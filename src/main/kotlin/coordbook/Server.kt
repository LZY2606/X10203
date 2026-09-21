package coordbook

import io.ktor.http.HttpStatusCode
import io.ktor.serialization.kotlinx.json.json
import io.ktor.server.application.Application
import io.ktor.server.application.install
import io.ktor.server.plugins.contentnegotiation.ContentNegotiation
import io.ktor.server.plugins.statuspages.StatusPages
import io.ktor.server.response.respond
import io.ktor.server.routing.routing
import kotlinx.serialization.Serializable
import kotlinx.serialization.json.Json

@Serializable
data class ErrorDto(val error: String, val type: String)

fun Application.appModule(db: Database, service: RobotService) {
    install(ContentNegotiation) {
        json(Json {
            prettyPrint = true
            ignoreUnknownKeys = true
            encodeDefaults = true
        })
    }
    install(StatusPages) {
        exception<XmlException> { call, cause ->
            call.respond(HttpStatusCode.BadRequest, ErrorDto(cause.message ?: "XML 错误", "xml"))
        }
        exception<IllegalArgumentException> { call, cause ->
            call.respond(HttpStatusCode.BadRequest, ErrorDto(cause.message ?: "参数错误", "argument"))
        }
        exception<NoSuchElementException> { call, cause ->
            call.respond(HttpStatusCode.NotFound, ErrorDto(cause.message ?: "资源不存在", "not_found"))
        }
        exception<OptimisticLockException> { call, cause ->
            call.respond(
                HttpStatusCode.Conflict,
                ErrorDto(cause.message ?: "版本号冲突，请刷新草案后重试", "version_conflict"),
            )
        }
    }
    routing {
        apiRoutes(service, db)
        staticWeb()
    }
}
