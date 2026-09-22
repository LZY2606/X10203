package atlas

import io.ktor.server.engine.embeddedServer
import io.ktor.server.netty.Netty

fun main(args: Array<String>) {
    var host = "127.0.0.1"
    var port = 5263
    var db = "atlas.db"
    var i = 0
    while (i < args.size) {
        val a = args[i]
        val eq = a.indexOf('=')
        val key = if (eq > 0) a.substring(0, eq) else a
        val value = if (eq > 0) a.substring(eq + 1) else args.getOrNull(i + 1).also { i++ } ?: ""
        when (key) {
            "--host" -> host = value
            "--port" -> port = value.toIntOrNull() ?: port
            "--db" -> db = value
        }
        i++
    }
    val atlas = Atlas(Store(db))
    println("关节坐标册 listening on http://$host:$port (db=$db)")
    embeddedServer(Netty, port = port, host = host) { module(atlas) }.start(wait = true)
}
