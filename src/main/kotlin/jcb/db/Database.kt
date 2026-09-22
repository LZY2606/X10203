package jcb.db

import jcb.calib.CalibStatus
import jcb.calib.CalibrationTransform
import java.sql.Connection
import java.sql.DriverManager
import java.time.Instant

data class RobotRecord(
    val id: Long,
    val serial: String,
    val name: String,
    val urdfXml: String,
    val importedAt: Instant,
)

data class SnapshotRecord(
    val id: Long,
    val robotId: Long,
    val label: String,
    val takenAt: Instant?,
    val jointValuesJson: String,
    val frozen: Boolean,
    val createdAt: Instant,
)

class Database(path: String = "") {
    val conn: Connection = if (path.isBlank())
        DriverManager.getConnection("jdbc:sqlite::memory:")
    else DriverManager.getConnection("jdbc:sqlite:$path")

    init {
        conn.createStatement().use { st ->
            st.executeUpdate("PRAGMA foreign_keys = ON")
            st.executeUpdate(
                """
                CREATE TABLE IF NOT EXISTS robot (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    serial TEXT NOT NULL UNIQUE,
                    name TEXT,
                    urdf_xml TEXT NOT NULL,
                    imported_at TEXT NOT NULL
                )""".trimIndent()
            )
            st.executeUpdate(
                """
                CREATE TABLE IF NOT EXISTS snapshot (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    robot_id INTEGER NOT NULL REFERENCES robot(id),
                    label TEXT NOT NULL,
                    taken_at TEXT,
                    joint_values_json TEXT NOT NULL,
                    frozen INTEGER NOT NULL DEFAULT 0,
                    created_at TEXT NOT NULL
                )""".trimIndent()
            )
            st.executeUpdate(
                """
                CREATE TABLE IF NOT EXISTS calibration (
                    id TEXT NOT NULL,
                    version INTEGER NOT NULL,
                    robot_serial TEXT NOT NULL,
                    parent_frame TEXT NOT NULL,
                    child_frame TEXT NOT NULL,
                    xyz TEXT NOT NULL,
                    rpy_rad TEXT NOT NULL,
                    note TEXT,
                    valid_from TEXT,
                    valid_to TEXT,
                    created_at TEXT NOT NULL,
                    status TEXT NOT NULL,
                    base_version INTEGER NOT NULL DEFAULT 0,
                    PRIMARY KEY (id, version)
                )""".trimIndent()
            )
            st.executeUpdate(
                """
                CREATE TABLE IF NOT EXISTS patch (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    calibration_id TEXT NOT NULL,
                    base_version INTEGER NOT NULL,
                    payload_json TEXT NOT NULL,
                    status TEXT NOT NULL DEFAULT 'draft',
                    created_at TEXT NOT NULL,
                    decided_at TEXT
                )""".trimIndent()
            )
        }
    }

    fun importRobot(serial: String, name: String?, xml: String): RobotRecord {
        val now = Instant.now().toString()
        conn.prepareStatement(
            "INSERT INTO robot(serial, name, urdf_xml, imported_at) VALUES(?,?,?,?)"
        ).use { ps ->
            ps.setString(1, serial); ps.setString(2, name)
            ps.setString(3, xml); ps.setString(4, now)
            ps.executeUpdate()
        }
        return getRobotBySerial(serial)!!
    }

    fun getRobotBySerial(serial: String): RobotRecord? =
        conn.prepareStatement("SELECT * FROM robot WHERE serial = ?").use { ps ->
            ps.setString(1, serial)
            ps.executeQuery().use { rs ->
                if (rs.next()) RobotRecord(
                    rs.getLong("id"), rs.getString("serial"),
                    rs.getString("name"), rs.getString("urdf_xml"),
                    Instant.parse(rs.getString("imported_at"))
                ) else null
            }
        }

    fun listRobots(): List<RobotRecord> =
        conn.createStatement().use { st ->
            st.executeQuery("SELECT * FROM robot ORDER BY id").use { rs ->
                val out = mutableListOf<RobotRecord>()
                while (rs.next()) out += RobotRecord(
                    rs.getLong("id"), rs.getString("serial"),
                    rs.getString("name"), rs.getString("urdf_xml"),
                    Instant.parse(rs.getString("imported_at"))
                )
                out
            }
        }

    fun insertSnapshot(
        robotId: Long, label: String, takenAt: Instant?, valuesJson: String, frozen: Boolean
    ): Long {
        return conn.prepareStatement(
            "INSERT INTO snapshot(robot_id, label, taken_at, joint_values_json, frozen, created_at) " +
                "VALUES(?,?,?,?,?,?)",
            java.sql.Statement.RETURN_GENERATED_KEYS,
        ).use { ps ->
            ps.setLong(1, robotId); ps.setString(2, label)
            ps.setString(3, takenAt?.toString()); ps.setString(4, valuesJson)
            ps.setInt(5, if (frozen) 1 else 0); ps.setString(6, Instant.now().toString())
            ps.executeUpdate()
            ps.generatedKeys.use { rs ->
                if (rs.next()) rs.getLong(1) else -1L
            }
        }
    }

    fun setSnapshotFrozen(id: Long, frozen: Boolean) {
        conn.prepareStatement("UPDATE snapshot SET frozen = ? WHERE id = ?").use { ps ->
            ps.setInt(1, if (frozen) 1 else 0); ps.setLong(2, id); ps.executeUpdate()
        }
    }

    fun listSnapshots(robotId: Long): List<SnapshotRecord> =
        conn.prepareStatement("SELECT * FROM snapshot WHERE robot_id = ? ORDER BY id").use { ps ->
            ps.setLong(1, robotId)
            ps.executeQuery().use { rs ->
                val out = mutableListOf<SnapshotRecord>()
                while (rs.next()) out += SnapshotRecord(
                    rs.getLong("id"), rs.getLong("robot_id"), rs.getString("label"),
                    rs.getString("taken_at")?.let(Instant::parse),
                    rs.getString("joint_values_json"),
                    rs.getInt("frozen") == 1,
                    Instant.parse(rs.getString("created_at"))
                )
                out
            }
        }

    fun getSnapshot(id: Long): SnapshotRecord? =
        conn.prepareStatement("SELECT * FROM snapshot WHERE id = ?").use { ps ->
            ps.setLong(1, id)
            ps.executeQuery().use { rs ->
                if (rs.next()) SnapshotRecord(
                    rs.getLong("id"), rs.getLong("robot_id"), rs.getString("label"),
                    rs.getString("taken_at")?.let(Instant::parse),
                    rs.getString("joint_values_json"),
                    rs.getInt("frozen") == 1,
                    Instant.parse(rs.getString("created_at"))
                ) else null
            }
        }

    fun saveCalibration(c: CalibrationTransform) {
        conn.prepareStatement(
            """
            INSERT INTO calibration(id, version, robot_serial, parent_frame, child_frame,
                xyz, rpy_rad, note, valid_from, valid_to, created_at, status, base_version)
            VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?)
            ON CONFLICT(id, version) DO UPDATE SET
                robot_serial=excluded.robot_serial,
                parent_frame=excluded.parent_frame,
                child_frame=excluded.child_frame,
                xyz=excluded.xyz, rpy_rad=excluded.rpy_rad,
                note=excluded.note, valid_from=excluded.valid_from, valid_to=excluded.valid_to,
                status=excluded.status, base_version=excluded.base_version
            """.trimIndent()
        ).use { ps ->
            ps.setString(1, c.id); ps.setInt(2, c.version)
            ps.setString(3, c.robotSerial); ps.setString(4, c.parentFrame)
            ps.setString(5, c.childFrame)
            ps.setString(6, c.xyz.joinToString(",")); ps.setString(7, c.rpyRad.joinToString(","))
            ps.setString(8, c.note); ps.setString(9, c.validFrom?.toString())
            ps.setString(10, c.validTo?.toString()); ps.setString(11, c.createdAt.toString())
            ps.setString(12, c.status.name); ps.setInt(13, c.baseVersion)
            ps.executeUpdate()
        }
    }

    fun listCalibrations(serial: String? = null): List<CalibrationTransform> {
        val sql = if (serial == null)
            "SELECT * FROM calibration ORDER BY id, version"
        else "SELECT * FROM calibration WHERE robot_serial = ? ORDER BY id, version"
        val ps = conn.prepareStatement(sql)
        if (serial != null) ps.setString(1, serial)
        return ps.use {
            it.executeQuery().use { rs ->
                val out = mutableListOf<CalibrationTransform>()
                while (rs.next()) out += CalibrationTransform(
                    id = rs.getString("id"),
                    version = rs.getInt("version"),
                    robotSerial = rs.getString("robot_serial"),
                    parentFrame = rs.getString("parent_frame"),
                    childFrame = rs.getString("child_frame"),
                    xyz = rs.getString("xyz").split(",").map { v -> v.toDouble() }.toDoubleArray(),
                    rpyRad = rs.getString("rpy_rad").split(",").map { v -> v.toDouble() }.toDoubleArray(),
                    note = rs.getString("note"),
                    validFrom = rs.getString("valid_from")?.let(Instant::parse),
                    validTo = rs.getString("valid_to")?.let(Instant::parse),
                    createdAt = Instant.parse(rs.getString("created_at")),
                    status = CalibStatus.valueOf(rs.getString("status")),
                    baseVersion = rs.getInt("base_version"),
                )
                out
            }
        }
    }

    fun latestApprovedCalibration(id: String): CalibrationTransform? =
        listCalibrations().filter { it.id == id && it.status == CalibStatus.APPROVED }
            .maxByOrNull { it.version }

    fun close() = conn.close()
}
