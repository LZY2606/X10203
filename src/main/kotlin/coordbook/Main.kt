package coordbook

import io.ktor.server.engine.embeddedServer
import io.ktor.server.netty.Netty
import java.io.File

fun main(args: Array<String>) {
    var host = "127.0.0.1"
    var port = 5263
    var dbPath = "coordbook.db"
    var i = 0
    while (i < args.size) {
        when (args[i]) {
            "--host" -> host = args[++i]
            "--port" -> port = args[++i].toInt()
            "--db" -> dbPath = args[++i]
            else -> throw IllegalArgumentException("未知参数 ${args[i]}，支持 --host/--port/--db")
        }
        i++
    }
    File(dbPath).absoluteFile.parentFile?.mkdirs()
    val db = Database(dbPath)
    val service = RobotService(db)
    service.seedIfEmpty()
    embeddedServer(Netty, port = port, host = host) {
        appModule(db, service)
    }.start(wait = true)
}
