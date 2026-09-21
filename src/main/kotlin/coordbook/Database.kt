package coordbook

import java.sql.Connection
import java.sql.DriverManager

data class Robot(
    val id: Long,
    val name: String,
    val serial: String,
    val urdfXml: String,
    val currentRevision: Int,
    val createdAt: String,
    val updatedAt: String,
)

class OptimisticLockException(message: String) : RuntimeException(message)

class Database(private val path: String) {
    val conn: Connection = DriverManager.getConnection("jdbc:sqlite:$path").apply {
        createStatement().execute("PRAGMA foreign_keys = ON")
    }

    init {
        migrate()
    }

    private fun migrate() {
        conn.createStatement().use { st ->
            st.executeUpdate(
                """
                CREATE TABLE IF NOT EXISTS robot (
                  id INTEGER PRIMARY KEY AUTOINCREMENT,
                  name TEXT NOT NULL,
                  serial TEXT NOT NULL UNIQUE,
                  urdf_xml TEXT NOT NULL,
                  current_revision INTEGER NOT NULL,
                  created_at TEXT NOT NULL,
                  updated_at TEXT NOT NULL
                )
                """.trimIndent(),
            )
            st.executeUpdate(
                """
                CREATE TABLE IF NOT EXISTS calib_version (
                  id INTEGER PRIMARY KEY AUTOINCREMENT,
                  robot_id INTEGER NOT NULL REFERENCES robot(id),
                  serial_scope TEXT NOT NULL,
                  valid_from TEXT NOT NULL,
                  valid_until TEXT NOT NULL,
                  revision INTEGER NOT NULL,
                  note TEXT,
                  created_at TEXT NOT NULL
                )
                """.trimIndent(),
            )
            st.executeUpdate(
                """
                CREATE TABLE IF NOT EXISTS calib_patch (
                  id INTEGER PRIMARY KEY AUTOINCREMENT,
                  version_id INTEGER NOT NULL REFERENCES calib_version(id),
                  kind TEXT NOT NULL,
                  target_joint TEXT,
                  parent_frame TEXT,
                  child_frame TEXT,
                  xyz TEXT,
                  rpy TEXT,
                  status TEXT NOT NULL,
                  note TEXT,
                  created_at TEXT NOT NULL
                )
                """.trimIndent(),
            )
            st.executeUpdate(
                """
                CREATE TABLE IF NOT EXISTS snapshot (
                  id INTEGER PRIMARY KEY AUTOINCREMENT,
                  robot_id INTEGER NOT NULL REFERENCES robot(id),
                  serial TEXT NOT NULL,
                  frozen_at TEXT NOT NULL,
                  joint_values_json TEXT NOT NULL,
                  note TEXT,
                  created_at TEXT NOT NULL
                )
                """.trimIndent(),
            )
        }
    }

    // ---- robots ----

    fun insertRobot(name: String, serial: String, urdf: String): Robot {
        val now = IsoTime.now()
        conn.prepareStatement(
            """INSERT INTO robot(name, serial, urdf_xml, current_revision, created_at, updated_at)
               VALUES(?,?,?,1,?,?)""",
        ).use {
            it.setString(1, name)
            it.setString(2, serial)
            it.setString(3, urdf)
            it.setString(4, now)
            it.setString(5, now)
            it.executeUpdate()
        }
        return getRobotBySerial(serial)!!
    }

    fun listRobots(): List<Robot> {
        val out = mutableListOf<Robot>()
        conn.createStatement().use { st ->
            st.executeQuery("SELECT * FROM robot ORDER BY id").use { rs ->
                while (rs.next()) out += robotRow(rs)
            }
        }
        return out
    }

    fun getRobot(id: Long): Robot? = conn.prepareStatement("SELECT * FROM robot WHERE id=?").use {
        it.setLong(1, id)
        it.executeQuery().use { rs -> if (rs.next()) robotRow(rs) else null }
    }

    fun getRobotBySerial(serial: String): Robot? =
        conn.prepareStatement("SELECT * FROM robot WHERE serial=?").use {
            it.setString(1, serial)
            it.executeQuery().use { rs -> if (rs.next()) robotRow(rs) else null }
        }

    /** 乐观锁：期望 revision 匹配才更新，否则 409 语义异常。 */
    fun updateUrdf(id: Long, urdf: String, expectedRevision: Int): Robot {
        val now = IsoTime.now()
        conn.prepareStatement(
            """UPDATE robot SET urdf_xml=?, current_revision=current_revision+1, updated_at=?
               WHERE id=? AND current_revision=?""",
        ).use {
            it.setString(1, urdf)
            it.setString(2, now)
            it.setLong(3, id)
            it.setInt(4, expectedRevision)
            val rows = it.executeUpdate()
            if (rows == 0) {
                throw OptimisticLockException(
                    "URDF 版本号冲突：期望 revision=$expectedRevision，实际已被其他编辑更新",
                )
            }
        }
        return getRobot(id)!!
    }

    private fun robotRow(rs: java.sql.ResultSet) = Robot(
        rs.getLong("id"),
        rs.getString("name"),
        rs.getString("serial"),
        rs.getString("urdf_xml"),
        rs.getInt("current_revision"),
        rs.getString("created_at"),
        rs.getString("updated_at"),
    )

    // ---- calibration versions ----

    fun insertCalibVersion(
        robotId: Long,
        serialScope: String,
        validFrom: String,
        validUntil: String,
        note: String?,
    ): CalibrationVersion {
        // 草案并发控制：同一机器人的新版本 revision 单调递增，创建即锁住下一个号
        val revision = synchronized(this) {
            val max = conn.prepareStatement(
                "SELECT COALESCE(MAX(revision),0)+1 FROM calib_version WHERE robot_id=?",
            ).use {
                it.setLong(1, robotId)
                it.executeQuery().use { rs -> rs.getInt(1) }
            }
            val now = IsoTime.now()
            conn.prepareStatement(
                """INSERT INTO calib_version(robot_id, serial_scope, valid_from, valid_until, revision, note, created_at)
                   VALUES(?,?,?,?,?,?,?)""",
            ).use {
                it.setLong(1, robotId)
                it.setString(2, serialScope)
                it.setString(3, validFrom)
                it.setString(4, validUntil)
                it.setInt(5, max)
                it.setString(6, note)
                it.setString(7, now)
                it.executeUpdate()
            }
            max
        }
        return listCalibVersions(robotId).first { it.revision == revision }
    }

    fun listCalibVersions(robotId: Long): List<CalibrationVersion> =
        conn.prepareStatement("SELECT * FROM calib_version WHERE robot_id=? ORDER BY revision").use {
            it.setLong(1, robotId)
            it.executeQuery().use { rs ->
                val out = mutableListOf<CalibrationVersion>()
                while (rs.next()) out += CalibrationVersion(
                    rs.getLong("id"),
                    robotId,
                    rs.getString("serial_scope"),
                    rs.getString("valid_from"),
                    rs.getString("valid_until"),
                    rs.getInt("revision"),
                    rs.getString("note"),
                    rs.getString("created_at"),
                )
                out
            }
        }

    fun getCalibVersion(id: Long): CalibrationVersion? =
        conn.prepareStatement("SELECT * FROM calib_version WHERE id=?").use {
            it.setLong(1, id)
            it.executeQuery().use { rs ->
                if (rs.next()) CalibrationVersion(
                    id, rs.getLong("robot_id"), rs.getString("serial_scope"),
                    rs.getString("valid_from"), rs.getString("valid_until"),
                    rs.getInt("revision"), rs.getString("note"), rs.getString("created_at"),
                ) else null
            }
        }

    // ---- patches ----

    fun insertPatch(
        versionId: Long,
        kind: PatchKind,
        targetJoint: String?,
        parentFrame: String?,
        childFrame: String?,
        xyz: DoubleArray?,
        rpy: DoubleArray?,
        note: String?,
    ): CalibPatch {
        val now = IsoTime.now()
        conn.prepareStatement(
            """INSERT INTO calib_patch(version_id, kind, target_joint, parent_frame, child_frame,
               xyz, rpy, status, note, created_at) VALUES(?,?,?,?,?,?,?,?,?,?)""",
        ).use {
            it.setLong(1, versionId)
            it.setString(2, kind.name)
            it.setString(3, targetJoint)
            it.setString(4, parentFrame)
            it.setString(5, childFrame)
            it.setString(6, xyz?.fmt())
            it.setString(7, rpy?.fmt())
            it.setString(8, PatchStatus.PENDING.name)
            it.setString(9, note)
            it.setString(10, now)
            it.executeUpdate()
        }
        return listPatches(versionId).maxBy { it.id }
    }

    fun listPatches(versionId: Long? = null): List<CalibPatch> {
        val sql = if (versionId == null) "SELECT * FROM calib_patch ORDER BY id"
        else "SELECT * FROM calib_patch WHERE version_id=? ORDER BY id"
        val out = mutableListOf<CalibPatch>()
        conn.prepareStatement(sql).use { st ->
            if (versionId != null) st.setLong(1, versionId)
            st.executeQuery().use { rs ->
                while (rs.next()) out += patchRow(rs)
            }
        }
        return out
    }

    /** 补丁审批带版本草案修订号乐观锁：调用方提供 expectedRevision（calib_version.revision 不变性占位）。 */
    fun setPatchStatus(
        patchId: Long,
        status: PatchStatus,
        expectedVersionRevision: Int,
    ): CalibPatch {
        val patch = listPatches().firstOrNull { it.id == patchId }
            ?: throw NoSuchElementException("补丁 #$patchId 不存在")
        val version = getCalibVersion(patch.versionId)!!
        if (version.revision != expectedVersionRevision) {
            throw OptimisticLockException(
                "标定草案版本号冲突：期望 v$expectedVersionRevision，当前 v${version.revision}",
            )
        }
        conn.prepareStatement("UPDATE calib_patch SET status=? WHERE id=?").use {
            it.setString(1, status.name)
            it.setLong(2, patchId)
            val rows = it.executeUpdate()
            if (rows == 0) throw NoSuchElementException("补丁 #$patchId 更新失败")
        }
        return listPatches(patch.versionId).first { it.id == patchId }
    }

    private fun patchRow(rs: java.sql.ResultSet) = CalibPatch(
        rs.getLong("id"),
        rs.getLong("version_id"),
        PatchKind.valueOf(rs.getString("kind")),
        rs.getString("target_joint"),
        rs.getString("parent_frame"),
        rs.getString("child_frame"),
        rs.getString("xyz")?.let(::parseTriplet),
        rs.getString("rpy")?.let(::parseTriplet),
        PatchStatus.valueOf(rs.getString("status")),
        rs.getString("note"),
        rs.getString("created_at"),
    )

    // ---- snapshots ----

    fun insertSnapshot(
        robotId: Long,
        serial: String,
        frozenAt: String,
        valuesJson: String,
        note: String?,
    ): Snapshot {
        val now = IsoTime.now()
        conn.prepareStatement(
            """INSERT INTO snapshot(robot_id, serial, frozen_at, joint_values_json, note, created_at)
               VALUES(?,?,?,?,?,?)""",
        ).use {
            it.setLong(1, robotId)
            it.setString(2, serial)
            it.setString(3, frozenAt)
            it.setString(4, valuesJson)
            it.setString(5, note)
            it.setString(6, now)
            it.executeUpdate()
        }
        return listSnapshots(robotId).maxBy { it.id }
    }

    fun listSnapshots(robotId: Long): List<Snapshot> =
        conn.prepareStatement("SELECT * FROM snapshot WHERE robot_id=? ORDER BY id").use {
            it.setLong(1, robotId)
            it.executeQuery().use { rs ->
                val out = mutableListOf<Snapshot>()
                while (rs.next()) {
                    val id = rs.getLong("id")
                    out += Snapshot(
                        id, robotId, rs.getString("serial"),
                        rs.getString("frozen_at"),
                        JsonMaps.decodeDoubles(rs.getString("joint_values_json")),
                        rs.getString("note"), rs.getString("created_at"),
                    )
                }
                out
            }
        }

    fun getSnapshot(id: Long): Snapshot? =
        conn.prepareStatement("SELECT * FROM snapshot WHERE id=?").use {
            it.setLong(1, id)
            it.executeQuery().use { rs ->
                if (rs.next()) Snapshot(
                    id, rs.getLong("robot_id"), rs.getString("serial"),
                    rs.getString("frozen_at"),
                    JsonMaps.decodeDoubles(rs.getString("joint_values_json")),
                    rs.getString("note"), rs.getString("created_at"),
                ) else null
            }
        }
}
