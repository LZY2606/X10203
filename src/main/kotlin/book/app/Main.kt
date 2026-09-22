package book.app

import book.db.Database
import book.web.WebServer
import java.io.File

fun main(args: Array<String>) {
    var host = "127.0.0.1"
    var port = 5263
    var dbPath = "book.sqlite"
    var seed = true
    var i = 0
    while (i < args.size) {
        when (args[i]) {
            "--host" -> { host = args[++i] }
            "--port" -> { port = args[++i].toInt() }
            "--db" -> { dbPath = args[++i] }
            "--no-seed" -> { seed = false }
            "--reset" -> { File(dbPath).delete() }
            else -> System.err.println("未知参数: ${args[i]}")
        }
        i++
    }
    val db = Database(dbPath)
    if (seed && db.listRobots().isEmpty()) {
        DemoData.seed(db)
        println("已播种演示数据（--no-seed 可关闭）")
    }
    println("关节坐标册启动于 http://$host:$port")
    WebServer(AppService(db)).start(host, port)
}
