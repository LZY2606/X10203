package jcb.db

import jcb.model.CalibrationDeclaration
import jcb.model.CalibrationVersion
import jcb.model.JointSnapshot
import java.time.Instant

data class RobotRow(val serial: String, val name: String, val createdAt: Instant)

data class UrdfVersionRow(
    val id: Long,
    val robotSerial: String,
    val version: Int,
    val originalXml: String,
    val importedAt: Instant,
)

enum class PatchStatus { PROPOSED, APPROVED, REJECTED }
enum class PatchOperation { SET_ORIGIN, SET_LIMIT, SET_AXIS, SET_ATTRIBUTE, ADD_ELEMENT, REMOVE_ELEMENT }
enum class PatchTargetType { JOINT, LINK, ROBOT }

data class PatchRow(
    val id: Long,
    val robotSerial: String,
    val urdfVersion: Int,
    val targetType: PatchTargetType,
    val targetName: String,
    val operation: PatchOperation,
    val payload: String,
    val status: PatchStatus,
    val baseRevision: Long,
    val createdAt: Instant,
    val decidedAt: Instant?,
)

class OptimisticLockException(message: String) : RuntimeException(message)

class Repository(private val db: Database) {

    fun createRobotIfAbsent(serial: String, name: String) {
        db.tx { conn ->
            conn.prepareStatement(
                "INSERT OR IGNORE INTO robots(serial, name, created_at) VALUES(?,?,?)"
            ).use { ps ->
                ps.setString(1, serial)
                ps.setString(2, name)
                ps.setString(3, Instant.now().toString())
                ps.executeUpdate()
            }
        }
    }

    fun listRobots(): List<RobotRow> = db.tx { conn ->
        conn.createStatement().executeQuery("SELECT serial, name, created_at FROM robots ORDER BY serial")
            .use { rs ->
                buildList {
                    while (rs.next()) {
                        add(RobotRow(rs.getString("serial"), rs.getString("name"),
                            Instant.parse(rs.getString("created_at"))))
                    }
                }
            }
    }

    fun getRobot(serial: String): RobotRow? = db.tx { conn ->
        conn.prepareStatement("SELECT serial, name, created_at FROM robots WHERE serial = ?").use { ps ->
            ps.setString(1, serial)
            ps.executeQuery().use { rs ->
                if (rs.next()) RobotRow(rs.getString("serial"), rs.getString("name"),
                    Instant.parse(rs.getString("created_at"))) else null
            }
        }
    }

    fun importUrdf(robotSerial: String, xml: String): UrdfVersionRow {
        val now = Instant.now()
        return db.tx { conn ->
            val nextVersion = conn.prepareStatement(
                "SELECT COALESCE(MAX(version), 0) + 1 FROM urdf_versions WHERE robot_serial = ?"
            ).use { ps ->
                ps.setString(1, robotSerial)
                ps.executeQuery().use { it.getInt(1) }
            }
            conn.prepareStatement(
                """INSERT INTO urdf_versions(robot_serial, version, original_xml, imported_at)
                   VALUES(?,?,?,?)"""
            ).use { ps ->
                ps.setString(1, robotSerial)
                ps.setInt(2, nextVersion)
                ps.setString(3, xml)
                ps.setString(4, now.toString())
                ps.executeUpdate()
            }
            conn.prepareStatement("SELECT id FROM urdf_versions WHERE robot_serial=? AND version=?").use { ps ->
                ps.setString(1, robotSerial)
                ps.setInt(2, nextVersion)
                ps.executeQuery().use { rs ->
                    rs.next()
                    UrdfVersionRow(rs.getLong(1), robotSerial, nextVersion, xml, now)
                }
            }
        }
    }

    fun latestUrdf(robotSerial: String): UrdfVersionRow? = db.tx { conn ->
        conn.prepareStatement(
            """SELECT id, robot_serial, version, original_xml, imported_at
              FROM urdf_versions WHERE robot_serial = ?
              ORDER BY version DESC LIMIT 1"""
        ).use { ps ->
            ps.setString(1, robotSerial)
            ps.executeQuery().use { rs ->
                if (rs.next()) UrdfVersionRow(
                    rs.getLong("id"), rs.getString("robot_serial"), rs.getInt("version"),
                    rs.getString("original_xml"), Instant.parse(rs.getString("imported_at")),
                ) else null
            }
        }
    }

    fun saveCalibration(version: CalibrationVersion, versionNumber: Int) {
        db.tx { conn ->
            conn.prepareStatement(
                """INSERT INTO calibration_versions(id, robot_serial, version, label,
                   valid_from, valid_until, note, created_at) VALUES(?,?,?,?,?,?,?,?)"""
            ).use { ps ->
                ps.setString(1, version.id)
                ps.setString(2, version.robotSerial)
                ps.setInt(3, versionNumber)
                ps.setString(4, version.label)
                ps.setString(5, version.validFrom?.toString())
                ps.setString(6, version.validUntil?.toString())
                ps.setString(7, version.note)
                ps.setString(8, version.createdAt.toString())
                ps.executeUpdate()
            }
            version.declarations.forEachIndexed { ordinal, decl ->
                conn.prepareStatement(
                    """INSERT INTO calibration_declarations(id, calibration_id, parent_frame,
                       child_frame, xyz_meters, rpy_radians, ordinal, comment)
                       VALUES(?,?,?,?,?,?,?,?)"""
                ).use { ps ->
                    ps.setString(1, decl.id)
                    ps.setString(2, version.id)
                    ps.setString(3, decl.parentFrame)
                    ps.setString(4, decl.childFrame)
                    ps.setString(5, decl.xyzMeters.joinToString(",") { it.toString() })
                    ps.setString(6, decl.rpyRadians.joinToString(",") { it.toString() })
                    ps.setInt(7, ordinal)
                    ps.setString(8, decl.comment)
                    ps.executeUpdate()
                }
            }
        }
    }

    fun listCalibrations(robotSerial: String): List<CalibrationVersion> = db.tx { conn ->
        conn.prepareStatement(
            "SELECT id FROM calibration_versions WHERE robot_serial=? ORDER BY version"
        ).use { ps ->
            ps.setString(1, robotSerial)
            ps.executeQuery().use { rs ->
                buildList {
                    while (rs.next()) add(rs.getString(1))
                }
            }
        }
    }.mapNotNull { calibrationById(it) }

    fun calibrationById(id: String): CalibrationVersion? = db.tx { conn ->
        conn.prepareStatement(
            """SELECT id, robot_serial, version, label, valid_from, valid_until, note, created_at
               FROM calibration_versions WHERE id = ?"""
        ).use { ps ->
            ps.setString(1, id)
            ps.executeQuery().use { rs ->
                if (!rs.next()) return@tx null
                val header = CalibrationVersion(
                    id = rs.getString("id"),
                    robotSerial = rs.getString("robot_serial"),
                    label = rs.getString("label"),
                    validFrom = rs.getString("valid_from")?.let { Instant.parse(it) },
                    validUntil = rs.getString("valid_until")?.let { Instant.parse(it) },
                    createdAt = Instant.parse(rs.getString("created_at")),
                    note = rs.getString("note"),
                    declarations = emptyList(),
                )
                val decls = conn.prepareStatement(
                    """SELECT id, parent_frame, child_frame, xyz_meters, rpy_radians, comment
                       FROM calibration_declarations WHERE calibration_id=? ORDER BY ordinal"""
                ).use { dps ->
                    dps.setString(1, id)
                    dps.executeQuery().use { drs ->
                        buildList {
                            while (drs.next()) {
                                add(CalibrationDeclaration(
                                    id = drs.getString("id"),
                                    parentFrame = drs.getString("parent_frame"),
                                    childFrame = drs.getString("child_frame"),
                                    xyzMeters = parseTriple(drs.getString("xyz_meters")),
                                    rpyRadians = parseTriple(drs.getString("rpy_radians")),
                                    comment = drs.getString("comment"),
                                ))
                            }
                        }
                    }
                }
                header.copy(declarations = decls)
            }
        }
    }

    fun saveSnapshot(snapshot: JointSnapshot) {
        val valuesJson = buildString {
            append('{')
            append(snapshot.values.entries.joinToString(",") { (k, v) ->
                "\"${k.replace("\"", "\\\"")}\":$v"
            })
            append('}')
        }
        db.tx { conn ->
            conn.prepareStatement(
                """INSERT OR REPLACE INTO snapshots(id, robot_serial, frozen_at,
                   values_json, note, base_calibration_id) VALUES(?,?,?,?,?,?)"""
            ).use { ps ->
                ps.setString(1, snapshot.id)
                ps.setString(2, snapshot.robotSerial)
                ps.setString(3, snapshot.frozenAt.toString())
                ps.setString(4, valuesJson)
                ps.setString(5, snapshot.note)
                ps.setString(6, null)
                ps.executeUpdate()
            }
        }
    }

    fun snapshotById(id: String): JointSnapshot? = db.tx { conn ->
        conn.prepareStatement(
            "SELECT id, robot_serial, frozen_at, values_json, note FROM snapshots WHERE id=?"
        ).use { ps ->
            ps.setString(1, id)
            ps.executeQuery().use { rs ->
                if (!rs.next()) return@tx null
                val json = rs.getString("values_json")
                JointSnapshot(
                    id = rs.getString("id"),
                    robotSerial = rs.getString("robot_serial"),
                    frozenAt = Instant.parse(rs.getString("frozen_at")),
                    values = parseSimpleJsonObject(json),
                    note = rs.getString("note"),
                )
            }
        }
    }

    fun listSnapshots(robotSerial: String): List<JointSnapshot> = db.tx { conn ->
        conn.prepareStatement("SELECT id FROM snapshots WHERE robot_serial=? ORDER BY frozen_at DESC").use { ps ->
            ps.setString(1, robotSerial)
            ps.executeQuery().use { rs ->
                buildList { while (rs.next()) add(rs.getString(1)) }
            }
        }
    }.mapNotNull { snapshotById(it) }

    // ---- 补丁草案：版本号乐观并发 ----

    private fun currentRevisionLocked(conn: java.sql.Connection, serial: String): Long {
        conn.prepareStatement(
            "INSERT OR IGNORE INTO urdf_revisions(robot_serial, current_revision) VALUES(?,0)"
        ).use { it.setString(1, serial); it.executeUpdate() }
        conn.prepareStatement(
            "SELECT current_revision FROM urdf_revisions WHERE robot_serial=?"
        ).use { ps ->
            ps.setString(1, serial)
            ps.executeQuery().use { rs -> rs.next(); return rs.getLong(1) }
        }
    }

    fun proposePatch(row: PatchRow): Long {
        return db.tx { conn ->
            val current = currentRevisionLocked(conn, row.robotSerial)
            if (row.baseRevision != current) {
                throw OptimisticLockException(
                    "草案基于修订号 ${row.baseRevision}，当前已为 $current，请刷新后重新编辑")
            }
            conn.prepareStatement(
                """INSERT INTO urdf_patches(robot_serial, urdf_version, target_type, target_name,
                   operation, payload, status, base_revision, created_at)
                   VALUES(?,?,?,?,?,?, 'PROPOSED', ?, ?)"""
            ).use { ps ->
                ps.setString(1, row.robotSerial)
                ps.setInt(2, row.urdfVersion)
                ps.setString(3, row.targetType.name)
                ps.setString(4, row.targetName)
                ps.setString(5, row.operation.name)
                ps.setString(6, row.payload)
                ps.setLong(7, row.baseRevision)
                ps.setString(8, Instant.now().toString())
                ps.executeUpdate()
            }
            conn.prepareStatement("SELECT last_insert_rowid()").executeQuery().use { it.getLong(1) }
        }
    }

    fun decidePatch(patchId: Long, approve: Boolean, expectedRevision: Long): PatchRow {
        return db.tx { conn ->
            val existing = listPatchesInternal(conn).firstOrNull { it.id == patchId }
                ?: throw IllegalArgumentException("补丁不存在")
            if (existing.status != PatchStatus.PROPOSED) {
                throw IllegalStateException("补丁已被处理：${existing.status}")
            }
            val serial = existing.robotSerial
            val current = currentRevisionLocked(conn, serial)
            if (expectedRevision != current) {
                throw OptimisticLockException(
                    "修订号已从 $expectedRevision 变为 $current，请重新审阅后再批准")
            }
            val newStatus = if (approve) PatchStatus.APPROVED else PatchStatus.REJECTED
            conn.prepareStatement(
                "UPDATE urdf_patches SET status=?, decided_at=? WHERE id=?"
            ).use { ps ->
                ps.setString(1, newStatus.name)
                ps.setString(2, Instant.now().toString())
                ps.setLong(3, patchId)
                ps.executeUpdate()
            }
            if (approve) {
                conn.prepareStatement(
                    "UPDATE urdf_revisions SET current_revision = current_revision + 1 WHERE robot_serial=?"
                ).use { it.setString(1, serial); it.executeUpdate() }
            }
            listPatchesInternal(conn).first { it.id == patchId }
        }
    }

    fun currentRevision(serial: String): Long = db.tx { conn -> currentRevisionLocked(conn, serial) }

    fun listPatches(robotSerial: String): List<PatchRow> = db.tx { conn ->
        listPatchesInternal(conn).filter { it.robotSerial == robotSerial }
    }

    private fun listPatchesInternal(conn: java.sql.Connection): List<PatchRow> {
        return conn.createStatement().executeQuery(
            """SELECT id, robot_serial, urdf_version, target_type, target_name, operation,
                      payload, status, base_revision, created_at, decided_at
               FROM urdf_patches ORDER BY id"""
        ).use { rs ->
            buildList {
                while (rs.next()) {
                    add(PatchRow(
                        id = rs.getLong("id"),
                        robotSerial = rs.getString("robot_serial"),
                        urdfVersion = rs.getInt("urdf_version"),
                        targetType = PatchTargetType.valueOf(rs.getString("target_type")),
                        targetName = rs.getString("target_name"),
                        operation = PatchOperation.valueOf(rs.getString("operation")),
                        payload = rs.getString("payload"),
                        status = PatchStatus.valueOf(rs.getString("status")),
                        baseRevision = rs.getLong("base_revision"),
                        createdAt = Instant.parse(rs.getString("created_at")),
                        decidedAt = rs.getString("decided_at")?.let { Instant.parse(it) },
                    ))
                }
            }
        }
    }

    private fun parseTriple(raw: String): DoubleArray =
        raw.split(",").map { it.trim().toDouble() }.toDoubleArray()

    private fun parseSimpleJsonObject(json: String): Map<String, Double> {
        val out = linkedMapOf<String, Double>()
        val body = json.trim().removePrefix("{").removeSuffix("}").trim()
        if (body.isEmpty()) return out
        body.split(",").forEach { pair ->
            val idx = pair.indexOf(':')
            val key = pair.substring(0, idx).trim().removeSurrounding("\"")
                .replace("\\\"", "\"").replace("\\\\", "\\")
            out[key] = pair.substring(idx + 1).trim().toDouble()
        }
        return out
    }
}
