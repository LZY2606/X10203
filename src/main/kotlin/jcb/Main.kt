package jcb

import jcb.web.WebServer

fun main(args: Array<String>) {
    var host = "127.0.0.1"
    var port = 5263
    var dbPath = "jcb.sqlite"
    var i = 0
    while (i < args.size) {
        when (args[i]) {
            "--host" -> host = args[++i]
            "--port" -> port = args[++i].toInt()
            "--db" -> dbPath = args[++i]
        }
        i++
    }
    println("关节坐标册 启动中: http://$host:$port （数据库 $dbPath）")
    WebServer(dbPath).start(host, port)
}
