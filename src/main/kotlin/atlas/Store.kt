package atlas

import java.sql.Connection
import java.sql.DriverManager

class Store(path: String) {
    val conn: Connection = DriverManager.getConnection("jdbc:sqlite:$path")

    init {
        conn.createStatement().use { st ->
            st.executeUpdate("""
                CREATE TABLE IF NOT EXISTS robots(
                  id TEXT PRIMARY KEY,
                  name TEXT NOT NULL,
                  serial TEXT NOT NULL DEFAULT '',
                  xml TEXT NOT NULL,
                  version INTEGER NOT NULL DEFAULT 1,
                  created_at TEXT NOT NULL DEFAULT (datetime('now'))
                )""".trimIndent())
            st.executeUpdate("""
                CREATE TABLE IF NOT EXISTS calibrations(
                  id INTEGER PRIMARY KEY AUTOINCREMENT,
                  robot_id TEXT NOT NULL,
                  serial TEXT NOT NULL,
                  parent_frame TEXT NOT NULL,
                  child_frame TEXT NOT NULL,
                  x REAL, y REAL, z REAL,
                  roll REAL, pitch REAL, yaw REAL,
                  version INTEGER NOT NULL,
                  valid_from TEXT NOT NULL,
                  valid_to TEXT NOT NULL,
                  approved INTEGER NOT NULL DEFAULT 1,
                  created_at TEXT NOT NULL DEFAULT (datetime('now'))
                )""".trimIndent())
            st.executeUpdate("""
                CREATE TABLE IF NOT EXISTS snapshots(
                  id INTEGER PRIMARY KEY AUTOINCREMENT,
                  robot_id TEXT NOT NULL,
                  name TEXT NOT NULL,
                  joint_values TEXT NOT NULL,
                  default_unit TEXT NOT NULL DEFAULT 'rad',
                  joint_units TEXT NOT NULL DEFAULT '',
                  calib_max_version INTEGER NOT NULL,
                  frozen_at TEXT NOT NULL DEFAULT (datetime('now'))
                )""".trimIndent())
            st.executeUpdate("""
                CREATE TABLE IF NOT EXISTS drafts(
                  id INTEGER PRIMARY KEY AUTOINCREMENT,
                  robot_id TEXT NOT NULL,
                  base_version INTEGER NOT NULL,
                  description TEXT NOT NULL DEFAULT '',
                  patch_xml TEXT NOT NULL,
                  status TEXT NOT NULL DEFAULT 'draft',
                  created_at TEXT NOT NULL DEFAULT (datetime('now'))
                )""".trimIndent())
        }
    }

    data class RobotRow(val id: String, val name: String, val serial: String, val xml: String, val version: Int)
    data class SnapshotRow(val id: Long, val robotId: String, val name: String,
                           val jointValues: Map<String, Double>, val defaultUnit: String,
                           val jointUnits: Map<String, String>, val calibMaxVersion: Int, val frozenAt: String)
    data class DraftRow(val id: Long, val robotId: String, val baseVersion: Int,
                        val description: String, val patchXml: String, val status: String)

    fun insertRobot(id: String, name: String, serial: String, xml: String) {
        val ps = conn.prepareStatement("INSERT INTO robots(id,name,serial,xml) VALUES(?,?,?,?)")
        ps.setString(1, id); ps.setString(2, name); ps.setString(3, serial); ps.setString(4, xml)
        ps.executeUpdate()
    }

    fun getRobot(id: String): RobotRow? {
        val ps = conn.prepareStatement("SELECT id,name,serial,xml,version FROM robots WHERE id=?")
        ps.setString(1, id)
        val rs = ps.executeQuery()
        return if (rs.next()) RobotRow(rs.getString(1), rs.getString(2), rs.getString(3), rs.getString(4), rs.getInt(5)) else null
    }

    fun listRobots(): List<RobotRow> {
        val rs = conn.createStatement().executeQuery("SELECT id,name,serial,xml,version FROM robots ORDER BY created_at")
        val out = mutableListOf<RobotRow>()
        while (rs.next()) out.add(RobotRow(rs.getString(1), rs.getString(2), rs.getString(3), rs.getString(4), rs.getInt(5)))
        return out
    }

    fun bumpRobotVersion(id: String, newXml: String, expectedVersion: Int): Boolean {
        val ps = conn.prepareStatement("UPDATE robots SET xml=?, version=version+1 WHERE id=? AND version=?")
        ps.setString(1, newXml); ps.setString(2, id); ps.setInt(3, expectedVersion)
        return ps.executeUpdate() == 1
    }

    fun addCalibration(c: Calibration): Long {
        val ps = conn.prepareStatement("""
            INSERT INTO calibrations(robot_id,serial,parent_frame,child_frame,x,y,z,roll,pitch,yaw,version,valid_from,valid_to,approved)
            VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?)""".trimIndent(), java.sql.Statement.RETURN_GENERATED_KEYS)
        ps.setString(1, c.robotId); ps.setString(2, c.serial)
        ps.setString(3, c.parentFrame); ps.setString(4, c.childFrame)
        ps.setDouble(5, c.xyz[0]); ps.setDouble(6, c.xyz[1]); ps.setDouble(7, c.xyz[2])
        ps.setDouble(8, c.rpy[0]); ps.setDouble(9, c.rpy[1]); ps.setDouble(10, c.rpy[2])
        ps.setInt(11, c.version); ps.setString(12, c.validFrom); ps.setString(13, c.validTo)
        ps.setInt(14, if (c.approved) 1 else 0)
        ps.executeUpdate()
        val rs = ps.generatedKeys
        return if (rs.next()) rs.getLong(1) else -1
    }

    fun listCalibrations(robotId: String): List<Calibration> {
        val ps = conn.prepareStatement("SELECT * FROM calibrations WHERE robot_id=? ORDER BY id")
        ps.setString(1, robotId)
        val rs = ps.executeQuery()
        val out = mutableListOf<Calibration>()
        while (rs.next()) out.add(calFrom(rs))
        return out
    }

    /** 指定序列号与时刻生效的标定（闭区间端点包含），每个 (parent,child) 取最高版本 */
    fun effectiveCalibrations(robotId: String, serial: String, atTime: String, maxVersion: Int = Int.MAX_VALUE): List<Calibration> {
        val ps = conn.prepareStatement("""
            SELECT * FROM calibrations
            WHERE robot_id=? AND serial=? AND approved=1 AND version<=?
              AND valid_from<=? AND valid_to>=?
            ORDER BY parent_frame, child_frame, version DESC""".trimIndent())
        ps.setString(1, robotId); ps.setString(2, serial); ps.setInt(3, maxVersion)
        ps.setString(4, atTime); ps.setString(5, atTime)
        val rs = ps.executeQuery()
        val out = mutableListOf<Calibration>()
        val seen = mutableSetOf<Pair<String, String>>()
        while (rs.next()) {
            val c = calFrom(rs)
            if (seen.add(c.parentFrame to c.childFrame)) out.add(c)
        }
        return out
    }

    private fun calFrom(rs: java.sql.ResultSet) = Calibration(
        rs.getLong("id"), rs.getString("robot_id"), rs.getString("serial"),
        rs.getString("parent_frame"), rs.getString("child_frame"),
        doubleArrayOf(rs.getDouble("x"), rs.getDouble("y"), rs.getDouble("z")),
        doubleArrayOf(rs.getDouble("roll"), rs.getDouble("pitch"), rs.getDouble("yaw")),
        rs.getInt("version"), rs.getString("valid_from"), rs.getString("valid_to"),
        rs.getInt("approved") == 1)

    fun maxCalibVersion(robotId: String): Int {
        val ps = conn.prepareStatement("SELECT COALESCE(MAX(version),0) FROM calibrations WHERE robot_id=? AND approved=1")
        ps.setString(1, robotId)
        val rs = ps.executeQuery()
        return if (rs.next()) rs.getInt(1) else 0
    }

    fun freezeSnapshot(robotId: String, name: String, snap: JointSnapshot): Long {
        val ps = conn.prepareStatement("""
            INSERT INTO snapshots(robot_id,name,joint_values,default_unit,joint_units,calib_max_version)
            VALUES(?,?,?,?,?,?)""".trimIndent(), java.sql.Statement.RETURN_GENERATED_KEYS)
        ps.setString(1, robotId); ps.setString(2, name)
        ps.setString(3, snap.values.entries.joinToString(";") { "${it.key}=${it.value}" })
        ps.setString(4, snap.defaultUnit)
        ps.setString(5, snap.jointUnits.entries.joinToString(";") { "${it.key}=${it.value}" })
        ps.setInt(6, maxCalibVersion(robotId))
        ps.executeUpdate()
        val rs = ps.generatedKeys
        return if (rs.next()) rs.getLong(1) else -1
    }

    fun getSnapshot(id: Long): SnapshotRow? {
        val ps = conn.prepareStatement("SELECT * FROM snapshots WHERE id=?")
        ps.setLong(1, id)
        val rs = ps.executeQuery()
        if (!rs.next()) return null
        fun parseMap(s: String, conv: (String) -> Any): Map<String, Any> =
            s.split(";").filter { it.contains("=") }
                .associate { it.substringBefore("=") to conv(it.substringAfter("=")) }
        @Suppress("UNCHECKED_CAST")
        return SnapshotRow(rs.getLong("id"), rs.getString("robot_id"), rs.getString("name"),
            parseMap(rs.getString("joint_values")) { it.toDouble() } as Map<String, Double>,
            rs.getString("default_unit"),
            parseMap(rs.getString("joint_units")) { it } as Map<String, String>,
            rs.getInt("calib_max_version"), rs.getString("frozen_at"))
    }

    fun listSnapshots(robotId: String): List<SnapshotRow> {
        val ps = conn.prepareStatement("SELECT id FROM snapshots WHERE robot_id=? ORDER BY id")
        ps.setString(1, robotId)
        val rs = ps.executeQuery()
        val out = mutableListOf<SnapshotRow>()
        while (rs.next()) getSnapshot(rs.getLong(1))?.let { out.add(it) }
        return out
    }

    fun createDraft(robotId: String, baseVersion: Int, description: String, patchXml: String): Long {
        val ps = conn.prepareStatement(
            "INSERT INTO drafts(robot_id,base_version,description,patch_xml) VALUES(?,?,?,?)",
            java.sql.Statement.RETURN_GENERATED_KEYS)
        ps.setString(1, robotId); ps.setInt(2, baseVersion); ps.setString(3, description); ps.setString(4, patchXml)
        ps.executeUpdate()
        val rs = ps.generatedKeys
        return if (rs.next()) rs.getLong(1) else -1
    }

    fun getDraft(id: Long): DraftRow? {
        val ps = conn.prepareStatement("SELECT * FROM drafts WHERE id=?")
        ps.setLong(1, id)
        val rs = ps.executeQuery()
        return if (rs.next()) DraftRow(rs.getLong("id"), rs.getString("robot_id"), rs.getInt("base_version"),
            rs.getString("description"), rs.getString("patch_xml"), rs.getString("status")) else null
    }

    fun listDrafts(robotId: String): List<DraftRow> {
        val ps = conn.prepareStatement("SELECT * FROM drafts WHERE robot_id=? ORDER BY id")
        ps.setString(1, robotId)
        val rs = ps.executeQuery()
        val out = mutableListOf<DraftRow>()
        while (rs.next()) out.add(DraftRow(rs.getLong("id"), rs.getString("robot_id"), rs.getInt("base_version"),
            rs.getString("description"), rs.getString("patch_xml"), rs.getString("status")))
        return out
    }

    fun markDraft(id: Long, status: String) {
        val ps = conn.prepareStatement("UPDATE drafts SET status=? WHERE id=?")
        ps.setString(1, status); ps.setLong(2, id)
        ps.executeUpdate()
    }
}
