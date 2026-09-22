package coordbook

import kotlinx.serialization.json.Json
import kotlinx.serialization.json.JsonObject
import kotlinx.serialization.json.double
import kotlinx.serialization.json.jsonObject
import kotlinx.serialization.json.jsonPrimitive
import java.sql.Connection
import java.sql.DriverManager

class Db(path: String) {
    val conn: Connection = DriverManager.getConnection("jdbc:sqlite:$path")

    init {
        conn.createStatement().use { st ->
            st.executeUpdate(
                """
                CREATE TABLE IF NOT EXISTS urdf_doc(
                  id INTEGER PRIMARY KEY AUTOINCREMENT,
                  robot_name TEXT NOT NULL,
                  serial TEXT NOT NULL,
                  version INTEGER NOT NULL,
                  xml TEXT NOT NULL,
                  created_at INTEGER NOT NULL
                )"""
            )
            st.executeUpdate(
                """
                CREATE TABLE IF NOT EXISTS calibration(
                  id INTEGER PRIMARY KEY AUTOINCREMENT,
                  serial TEXT NOT NULL,
                  name TEXT NOT NULL,
                  parent_frame TEXT NOT NULL,
                  child_frame TEXT NOT NULL,
                  x REAL, y REAL, z REAL,
                  roll REAL, pitch REAL, yaw REAL,
                  angle_unit TEXT NOT NULL DEFAULT 'rad',
                  valid_from INTEGER, valid_to INTEGER,
                  version INTEGER NOT NULL,
                  status TEXT NOT NULL DEFAULT 'approved',
                  created_at INTEGER NOT NULL
                )"""
            )
            st.executeUpdate(
                """
                CREATE TABLE IF NOT EXISTS draft(
                  id INTEGER PRIMARY KEY AUTOINCREMENT,
                  doc_id INTEGER NOT NULL,
                  base_version INTEGER NOT NULL,
                  patch_json TEXT NOT NULL,
                  status TEXT NOT NULL DEFAULT 'pending',
                  created_at INTEGER NOT NULL
                )"""
            )
            st.executeUpdate(
                """
                CREATE TABLE IF NOT EXISTS snapshot(
                  id INTEGER PRIMARY KEY AUTOINCREMENT,
                  name TEXT NOT NULL,
                  serial TEXT NOT NULL,
                  joint_values TEXT NOT NULL,
                  time INTEGER,
                  frozen_at INTEGER NOT NULL
                )"""
            )
        }
    }

    fun now() = System.currentTimeMillis() / 1000

    // ---------- URDF docs ----------
    data class UrdfDocRow(val id: Long, val robotName: String, val serial: String, val version: Int, val xml: String)

    fun insertUrdf(robotName: String, serial: String, version: Int, xml: String): Long {
        val ps = conn.prepareStatement(
            "INSERT INTO urdf_doc(robot_name,serial,version,xml,created_at) VALUES(?,?,?,?,?)",
            java.sql.Statement.RETURN_GENERATED_KEYS,
        )
        ps.setString(1, robotName); ps.setString(2, serial); ps.setInt(3, version); ps.setString(4, xml)
        ps.setLong(5, now())
        ps.executeUpdate()
        val rs = ps.generatedKeys; rs.next(); return rs.getLong(1)
    }

    fun latestUrdf(serial: String): UrdfDocRow? {
        val ps = conn.prepareStatement("SELECT * FROM urdf_doc WHERE serial=? ORDER BY version DESC LIMIT 1")
        ps.setString(1, serial)
        val rs = ps.executeQuery()
        return if (rs.next()) UrdfDocRow(rs.getLong("id"), rs.getString("robot_name"), serial, rs.getInt("version"), rs.getString("xml")) else null
    }

    fun urdfVersions(serial: String): List<Pair<Int, Long>> {
        val ps = conn.prepareStatement("SELECT version, id FROM urdf_doc WHERE serial=? ORDER BY version")
        ps.setString(1, serial)
        val rs = ps.executeQuery()
        val out = mutableListOf<Pair<Int, Long>>()
        while (rs.next()) out.add(rs.getInt("version") to rs.getLong("id"))
        return out
    }

    fun urdfById(id: Long): UrdfDocRow? {
        val ps = conn.prepareStatement("SELECT * FROM urdf_doc WHERE id=?")
        ps.setLong(1, id)
        val rs = ps.executeQuery()
        return if (rs.next()) UrdfDocRow(id, rs.getString("robot_name"), rs.getString("serial"), rs.getInt("version"), rs.getString("xml")) else null
    }

    // ---------- Calibrations ----------
    fun insertCalibration(c: Calibration): Long {
        val ps = conn.prepareStatement(
            """INSERT INTO calibration(serial,name,parent_frame,child_frame,x,y,z,roll,pitch,yaw,angle_unit,valid_from,valid_to,version,status,created_at)
               VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)""",
            java.sql.Statement.RETURN_GENERATED_KEYS,
        )
        ps.setString(1, c.serial); ps.setString(2, c.name); ps.setString(3, c.parentFrame); ps.setString(4, c.childFrame)
        ps.setDouble(5, c.xyz[0]); ps.setDouble(6, c.xyz[1]); ps.setDouble(7, c.xyz[2])
        ps.setDouble(8, c.rpy[0]); ps.setDouble(9, c.rpy[1]); ps.setDouble(10, c.rpy[2])
        ps.setString(11, c.angleUnit)
        if (c.validFrom != null) ps.setLong(12, c.validFrom) else ps.setNull(12, java.sql.Types.INTEGER)
        if (c.validTo != null) ps.setLong(13, c.validTo) else ps.setNull(13, java.sql.Types.INTEGER)
        ps.setInt(14, c.version); ps.setString(15, c.status); ps.setLong(16, now())
        ps.executeUpdate()
        val rs = ps.generatedKeys; rs.next(); return rs.getLong(1)
    }

    fun calibrations(serial: String? = null): List<Calibration> {
        val sql = if (serial == null) "SELECT * FROM calibration ORDER BY name, version"
        else "SELECT * FROM calibration WHERE serial=? ORDER BY name, version"
        val ps = conn.prepareStatement(sql)
        if (serial != null) ps.setString(1, serial)
        val rs = ps.executeQuery()
        val out = mutableListOf<Calibration>()
        while (rs.next()) {
            out.add(
                Calibration(
                    id = rs.getLong("id"), serial = rs.getString("serial"), name = rs.getString("name"),
                    parentFrame = rs.getString("parent_frame"), childFrame = rs.getString("child_frame"),
                    xyz = doubleArrayOf(rs.getDouble("x"), rs.getDouble("y"), rs.getDouble("z")),
                    rpy = doubleArrayOf(rs.getDouble("roll"), rs.getDouble("pitch"), rs.getDouble("yaw")),
                    angleUnit = rs.getString("angle_unit"),
                    validFrom = rs.getObject("valid_from")?.let { (it as Number).toLong() },
                    validTo = rs.getObject("valid_to")?.let { (it as Number).toLong() },
                    version = rs.getInt("version"), status = rs.getString("status"),
                )
            )
        }
        return out
    }

    // ---------- Drafts (optimistic concurrency) ----------
    data class DraftRow(val id: Long, val docId: Long, val baseVersion: Int, val patchJson: String, val status: String)

    fun insertDraft(docId: Long, baseVersion: Int, patchJson: String): Long {
        val ps = conn.prepareStatement(
            "INSERT INTO draft(doc_id,base_version,patch_json,status,created_at) VALUES(?,?,?,'pending',?)",
            java.sql.Statement.RETURN_GENERATED_KEYS,
        )
        ps.setLong(1, docId); ps.setInt(2, baseVersion); ps.setString(3, patchJson); ps.setLong(4, now())
        ps.executeUpdate()
        val rs = ps.generatedKeys; rs.next(); return rs.getLong(1)
    }

    fun drafts(): List<DraftRow> {
        val rs = conn.createStatement().executeQuery("SELECT * FROM draft ORDER BY id")
        val out = mutableListOf<DraftRow>()
        while (rs.next()) out.add(DraftRow(rs.getLong("id"), rs.getLong("doc_id"), rs.getInt("base_version"), rs.getString("patch_json"), rs.getString("status")))
        return out
    }

    /** Approve a draft only if its base version still matches the doc's current version. */
    fun approveDraft(draftId: Long): String {
        val d = drafts().firstOrNull { it.id == draftId } ?: return "not-found"
        if (d.status != "pending") return "already-${d.status}"
        val doc = urdfById(d.docId) ?: return "not-found"
        val latest = latestUrdf(doc.serial) ?: return "not-found"
        if (latest.version != d.baseVersion) return "conflict"
        conn.prepareStatement("UPDATE draft SET status='approved' WHERE id=?").use { it.setLong(1, draftId); it.executeUpdate() }
        return "approved"
    }

    fun rejectDraft(draftId: Long): String {
        val d = drafts().firstOrNull { it.id == draftId } ?: return "not-found"
        conn.prepareStatement("UPDATE draft SET status='rejected' WHERE id=?").use { it.setLong(1, draftId); it.executeUpdate() }
        return "rejected"
    }

    // ---------- Snapshots ----------
    data class SnapshotRow(val id: Long, val name: String, val serial: String, val jointValues: Map<String, Double>, val time: Long?)

    fun insertSnapshot(name: String, serial: String, jointValues: Map<String, Double>, time: Long?): Long {
        val json = JsonObject(jointValues.mapValues { kotlinx.serialization.json.JsonPrimitive(it.value) })
        val ps = conn.prepareStatement(
            "INSERT INTO snapshot(name,serial,joint_values,time,frozen_at) VALUES(?,?,?,?,?)",
            java.sql.Statement.RETURN_GENERATED_KEYS,
        )
        ps.setString(1, name); ps.setString(2, serial); ps.setString(3, json.toString())
        if (time != null) ps.setLong(4, time) else ps.setNull(4, java.sql.Types.INTEGER)
        ps.setLong(5, now())
        ps.executeUpdate()
        val rs = ps.generatedKeys; rs.next(); return rs.getLong(1)
    }

    fun snapshots(): List<SnapshotRow> {
        val rs = conn.createStatement().executeQuery("SELECT * FROM snapshot ORDER BY id")
        val out = mutableListOf<SnapshotRow>()
        while (rs.next()) {
            val jv = Json.parseToJsonElement(rs.getString("joint_values")).jsonObject
                .mapValues { it.value.jsonPrimitive.double }
            out.add(SnapshotRow(rs.getLong("id"), rs.getString("name"), rs.getString("serial"), jv, rs.getObject("time")?.let { (it as Number).toLong() }))
        }
        return out
    }

    fun snapshot(id: Long): SnapshotRow? = snapshots().firstOrNull { it.id == id }
}
