package coordbook

import io.ktor.http.ContentType
import io.ktor.server.application.call
import io.ktor.server.http.content.staticResources
import io.ktor.server.response.respondText
import io.ktor.server.routing.Route
import io.ktor.server.routing.get

fun Route.staticWeb() {
    get("/") {
        val html = Route::class.java.getResource("/web/index.html")?.readText()
            ?: error("index.html 未打包")
        call.respondText(html, ContentType.Text.Html)
    }
    staticResources("/static", "web")
}
