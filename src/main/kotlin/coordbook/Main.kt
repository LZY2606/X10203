package coordbook

fun main(args: Array<String>) {
    var host = "127.0.0.1"
    var port = 5263
    var dbPath = System.getenv("COORDBOOK_DB") ?: "coordbook.db"
    var i = 0
    while (i < args.size) {
        when (args[i]) {
            "--host" -> { host = args[i + 1]; i += 2 }
            "--port" -> { port = args[i + 1].toInt(); i += 2 }
            "--db" -> { dbPath = args[i + 1]; i += 2 }
            else -> i++
        }
    }
    val db = Db(dbPath)
    Seed.seedIfEmpty(db)
    println("关节坐标册 listening on http://$host:$port (db=$dbPath)")
    createServer(db, host, port).start(wait = true)
}
