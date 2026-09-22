package book.db

import book.calib.CalibrationEdge
import book.calib.CalibrationVersion
import kotlinx.serialization.builtins.ListSerializer
import kotlinx.serialization.json.Json
import java.sql.Connection
import java.sql.DriverManager
import java.time.Instant
import java.util.UUID

data class RobotRecord(
    val id: String,
    val serial: String,
    val name: String,
    val urdfXml: String,
    val createdAt: String,
)

data class SnapshotRecord(
    val id: String,
    val robotId: String,
    val serial: String,
    val label: String,
    val frozenAt: String,
    val calibrationVersionId: String?,
    val jointValuesJson: String,
    val jointUnitsJson: String,
    val note: String,
)

data class ExportPatchRecord(
    val id: String,
    val robotId: String,
    val calibrationVersionId: String,
    val edgeIdsJson: String,
    val approved: Boolean,
    val createdAt: String,
    val note: String,
)

class Database(path: String) {
    private val conn: Connection = DriverManager.getConnection("jdbc:sqlite:$path")
    private val json = Json { ignoreUnknownKeys = true }

    init {
        conn.createStatement().use { st ->
            st.executeUpdate("PRAGMA foreign_keys = ON")
            st.executeUpdate(
                """
                CREATE TABLE IF NOT EXISTS robots (
                    id TEXT PRIMARY KEY,
                    serial TEXT NOT NULL UNIQUE,
                    name TEXT NOT NULL,
                    urdf_xml TEXT NOT NULL,
                    created_at TEXT NOT NULL
                )""".trimIndent()
            )
            st.executeUpdate(
                """
                CREATE TABLE IF NOT EXISTS calib_versions (
                    id TEXT PRIMARY KEY,
                    robot_id TEXT NOT NULL REFERENCES robots(id),
                    serial TEXT NOT NULL,
                    version INTEGER NOT NULL,
                    note TEXT NOT NULL,
                    created_at TEXT NOT NULL,
                    valid_from TEXT NOT NULL,
                    valid_until TEXT NOT NULL,
                    draft INTEGER NOT NULL,
                    approved INTEGER NOT NULL,
                    edges_json TEXT NOT NULL,
                    lock_version INTEGER NOT NULL DEFAULT 0,
                    UNIQUE(robot_id, version)
                )""".trimIndent()
            )
            st.executeUpdate(
                """
                CREATE TABLE IF NOT EXISTS snapshots (
                    id TEXT PRIMARY KEY,
                    robot_id TEXT NOT NULL REFERENCES robots(id),
                    serial TEXT NOT NULL,
                    label TEXT NOT NULL,
                    frozen_at TEXT NOT NULL,
                    calib_version_id TEXT,
                    joint_values_json TEXT NOT NULL,
                    joint_units_json TEXT NOT NULL,
                    note TEXT NOT NULL
                )""".trimIndent()
            )
            st.executeUpdate(
                """
                CREATE TABLE IF NOT EXISTS export_patches (
                    id TEXT PRIMARY KEY,
                    robot_id TEXT NOT NULL REFERENCES robots(id),
                    calib_version_id TEXT NOT NULL REFERENCES calib_versions(id),
                    edge_ids_json TEXT NOT NULL,
                    approved INTEGER NOT NULL,
                    created_at TEXT NOT NULL,
                    note TEXT NOT NULL
                )""".trimIndent()
            )
        }
    }

    fun close() = conn.close()

    // ---- robots ----

    fun upsertRobot(serial: String, name: String, urdfXml: String): RobotRecord {
        val existing = findRobotBySerial(serial)
        if (existing != null) {
            conn.prepareStatement("UPDATE robots SET name=?, urdf_xml=? WHERE id=?").use {
                it.setString(1, name); it.setString(2, urdfXml); it.setString(3, existing.id)
                it.executeUpdate()
            }
            return existing.copy(name = name, urdfXml = urdfXml)
        }
        val rec = RobotRecord(UUID.randomUUID().toString(), serial, name, urdfXml,
            Instant.now().toString())
        conn.prepareStatement(
            "INSERT INTO robots(id, serial, name, urdf_xml, created_at) VALUES(?,?,?,?,?)"
        ).use {
            it.setString(1, rec.id); it.setString(2, serial); it.setString(3, name)
            it.setString(4, urdfXml); it.setString(5, rec.createdAt)
            it.executeUpdate()
        }
        return rec
    }

    fun findRobotBySerial(serial: String): RobotRecord? =
        conn.prepareStatement("SELECT id, serial, name, urdf_xml, created_at FROM robots WHERE serial=?")
            .use { st ->
                st.setString(1, serial)
                st.executeQuery().use { rs ->
                    if (rs.next()) RobotRecord(
                        rs.getString(1), rs.getString(2), rs.getString(3),
                        rs.getString(4), rs.getString(5),
                    ) else null
                }
            }

    fun listRobots(): List<RobotRecord> =
        conn.createStatement().executeQuery(
            "SELECT id, serial, name, urdf_xml, created_at FROM robots ORDER BY serial"
        ).use { rs ->
            val out = mutableListOf<RobotRecord>()
            while (rs.next()) out += RobotRecord(
                rs.getString(1), rs.getString(2), rs.getString(3),
                rs.getString(4), rs.getString(5),
            )
            out
        }

    // ---- calibration versions ----

    fun insertCalibrationVersion(v: CalibrationVersion, robotId: String) {
        requireVersionFree(robotId, v.version, null)
        conn.prepareStatement(
            """INSERT INTO calib_versions
              |(id, robot_id, serial, version, note, created_at, valid_from, valid_until,
              | draft, approved, edges_json, lock_version)
              |VALUES(?,?,?,?,?,?,?,?,?,?,?,?)""".trimMargin()
        ).use {
            it.setString(1, v.id); it.setString(2, robotId); it.setString(3, v.robotSerial)
            it.setInt(4, v.version); it.setString(5, v.note); it.setString(6, v.createdAt)
            it.setString(7, v.validFrom); it.setString(8, v.validUntil)
            it.setInt(9, if (v.draft) 1 else 0); it.setInt(10, if (v.approved) 1 else 0)
            it.setString(11, json.encodeToString(ListSerializer(CalibrationEdge.serializer()), v.edges))
            it.setInt(12, 0)
            it.executeUpdate()
        }
    }

    /**
     * 乐观锁：业务版本号 version 不可变；每次成功编辑 lock_version +1。
     * expectedLockVersion 与库中不一致即拒绝写入。
     */
    fun lockVersion(versionId: String): Int =
        conn.prepareStatement("SELECT lock_version FROM calib_versions WHERE id=?").use { st ->
            st.setString(1, versionId)
            st.executeQuery().use { if (it.next()) it.getInt(1) else error("标定版本不存在") }
        }

    fun updateCalibrationVersion(
        versionId: String,
        expectedLockVersion: Int,
        note: String,
        draft: Boolean,
        approved: Boolean,
        edges: List<CalibrationEdge>,
    ): CalibrationVersion {
        conn.prepareStatement(
            """UPDATE calib_versions
              |SET note=?, draft=?, approved=?, edges_json=?, lock_version=lock_version+1
              |WHERE id=? AND lock_version=?""".trimMargin()
        ).use {
            it.setString(1, note); it.setInt(2, if (draft) 1 else 0)
            it.setInt(3, if (approved) 1 else 0)
            it.setString(4, json.encodeToString(ListSerializer(CalibrationEdge.serializer()), edges))
            it.setString(5, versionId); it.setInt(6, expectedLockVersion)
            val rows = it.executeUpdate()
            if (rows == 0) {
                throw ConcurrentModificationException(
                    "标定编辑并发冲突：期望 lock_version=$expectedLockVersion，" +
                        "当前为 ${lockVersion(versionId)}；请刷新后重试")
            }
        }
        return getCalibrationVersion(versionId)!!
    }

    fun nextCalibrationVersionNumber(robotId: String): Int =
        conn.prepareStatement("SELECT COALESCE(MAX(version),0)+1 FROM calib_versions WHERE robot_id=?")
            .use { st ->
                st.setString(1, robotId)
                st.executeQuery().use { if (it.next()) it.getInt(1) else 1 }
            }

    private fun requireVersionFree(robotId: String, version: Int, excludeId: String?) {
        conn.prepareStatement(
            "SELECT id FROM calib_versions WHERE robot_id=? AND version=?" +
                if (excludeId != null) " AND id<>?" else ""
        ).use { st ->
            st.setString(1, robotId); st.setInt(2, version)
            if (excludeId != null) st.setString(3, excludeId)
            st.executeQuery().use { rs ->
                if (rs.next()) throw ConcurrentModificationException(
                    "标定 v$version 已存在（并发创建冲突）")
            }
        }
    }

    fun listCalibrationVersions(robotId: String): List<CalibrationVersion> =
        conn.prepareStatement(
            """SELECT id, serial, version, note, created_at, valid_from, valid_until,
              |draft, approved, edges_json FROM calib_versions
              |WHERE robot_id=? ORDER BY version DESC""".trimMargin()
        ).use { st ->
            st.setString(1, robotId)
            st.executeQuery().use { rs ->
                val out = mutableListOf<CalibrationVersion>()
                while (rs.next()) out += rowToVersion(rs)
                out
            }
        }

    fun getCalibrationVersion(id: String): CalibrationVersion? =
        conn.prepareStatement(
            """SELECT id, serial, version, note, created_at, valid_from, valid_until,
              |draft, approved, edges_json FROM calib_versions WHERE id=?""".trimMargin()
        ).use { st ->
            st.setString(1, id)
            st.executeQuery().use { rs -> if (rs.next()) rowToVersion(rs) else null }
        }

    private fun rowToVersion(rs: java.sql.ResultSet): CalibrationVersion {
        val edges = json.decodeFromString(
            ListSerializer(CalibrationEdge.serializer()), rs.getString(10)
        )
        return CalibrationVersion(
            id = rs.getString(1),
            robotSerial = rs.getString(2),
            version = rs.getInt(3),
            note = rs.getString(4),
            createdAt = rs.getString(5),
            validFrom = rs.getString(6),
            validUntil = rs.getString(7),
            draft = rs.getInt(8) == 1,
            approved = rs.getInt(9) == 1,
            edges = edges,
        )
    }

    // ---- snapshots ----

    fun insertSnapshot(s: SnapshotRecord) {
        conn.prepareStatement(
            """INSERT INTO snapshots
              |(id, robot_id, serial, label, frozen_at, calib_version_id,
              | joint_values_json, joint_units_json, note)
              |VALUES(?,?,?,?,?,?,?,?,?)""".trimMargin()
        ).use {
            it.setString(1, s.id); it.setString(2, s.robotId); it.setString(3, s.serial)
            it.setString(4, s.label); it.setString(5, s.frozenAt)
            it.setString(6, s.calibrationVersionId)
            it.setString(7, s.jointValuesJson); it.setString(8, s.jointUnitsJson)
            it.setString(9, s.note)
            it.executeUpdate()
        }
    }

    fun listSnapshots(robotId: String): List<SnapshotRecord> =
        conn.prepareStatement(
            """SELECT id, robot_id, serial, label, frozen_at, calib_version_id,
              |joint_values_json, joint_units_json, note
              |FROM snapshots WHERE robot_id=? ORDER BY frozen_at DESC""".trimMargin()
        ).use { st ->
            st.setString(1, robotId)
            st.executeQuery().use { rs ->
                val out = mutableListOf<SnapshotRecord>()
                while (rs.next()) out += SnapshotRecord(
                    rs.getString(1), rs.getString(2), rs.getString(3), rs.getString(4),
                    rs.getString(5), rs.getString(6), rs.getString(7), rs.getString(8),
                    rs.getString(9),
                )
                out
            }
        }

    fun getSnapshot(id: String): SnapshotRecord? =
        conn.prepareStatement(
            """SELECT id, robot_id, serial, label, frozen_at, calib_version_id,
              |joint_values_json, joint_units_json, note
              |FROM snapshots WHERE id=?""".trimMargin()
        ).use { st ->
            st.setString(1, id)
            st.executeQuery().use { rs ->
                if (rs.next()) SnapshotRecord(
                    rs.getString(1), rs.getString(2), rs.getString(3), rs.getString(4),
                    rs.getString(5), rs.getString(6), rs.getString(7), rs.getString(8),
                    rs.getString(9),
                ) else null
            }
        }

    // ---- export patches ----

    fun insertExportPatch(p: ExportPatchRecord) {
        conn.prepareStatement(
            """INSERT INTO export_patches
              |(id, robot_id, calibration_version_id, edge_ids_json, approved, created_at, note)
              |VALUES(?,?,?,?,?,?,?)""".trimMargin()
        ).use {
            it.setString(1, p.id); it.setString(2, p.robotId)
            it.setString(3, p.calibrationVersionId); it.setString(4, p.edgeIdsJson)
            it.setInt(5, if (p.approved) 1 else 0); it.setString(6, p.createdAt)
            it.setString(7, p.note)
            it.executeUpdate()
        }
    }

    fun approvePatch(patchId: String): ExportPatchRecord? {
        conn.prepareStatement("UPDATE export_patches SET approved=1 WHERE id=?").use {
            it.setString(1, patchId); it.executeUpdate()
        }
        return listExportPatches(null).firstOrNull { it.id == patchId }
    }

    fun listExportPatches(robotId: String?): List<ExportPatchRecord> {
        val sql = "SELECT id, robot_id, calibration_version_id, edge_ids_json, approved, created_at, note FROM export_patches" +
            if (robotId != null) " WHERE robot_id=?" else ""
        return conn.prepareStatement(sql).use { st ->
            if (robotId != null) st.setString(1, robotId)
            st.executeQuery().use { rs ->
                val out = mutableListOf<ExportPatchRecord>()
                while (rs.next()) out += ExportPatchRecord(
                    rs.getString(1), rs.getString(2), rs.getString(3), rs.getString(4),
                    rs.getInt(5) == 1, rs.getString(6), rs.getString(7),
                )
                out
            }
        }
    }
}
