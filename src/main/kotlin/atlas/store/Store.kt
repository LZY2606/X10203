package atlas.store

import atlas.kinematics.CalibrationSet
import atlas.kinematics.CalibrationTransform
import kotlinx.serialization.Serializable
import kotlinx.serialization.json.Json
import java.sql.Connection
import java.sql.DriverManager

@Serializable
data class Snapshot(
    val id: Long,
    val name: String,
    val robotSerial: String?,
    val jointValues: Map<String, Double>,
    val capturedAt: Long,
    val calibrationId: Long?
)

@Serializable
data class PatchOp(
    val type: String,                 // "setOrigin" | "setAxis" | "setLimit"
    val joint: String,
    val xyz: List<Double>? = null,
    val rpy: List<Double>? = null,
    val axis: List<Double>? = null,
    val lower: Double? = null,
    val upper: Double? = null
)

@Serializable
data class Draft(
    val id: Long,
    val urdfId: Long,
    val baseVersion: Int,
    val ops: List<PatchOp>,
    val status: String,               // DRAFT | APPROVED | REJECTED
    val createdAt: Long
)

@Serializable
data class UrdfDoc(val id: Long, val name: String, val version: Int, val xml: String, val createdAt: Long)

class Store(path: String) : AutoCloseable {
    private val conn: Connection = DriverManager.getConnection("jdbc:sqlite:$path")
    private val json = Json { ignoreUnknownKeys = true }

    init {
        conn.createStatement().use { st ->
            // execute statements one by one (sqlite driver disallows multi-statement)
            for (sql in listOf(
                "CREATE TABLE IF NOT EXISTS urdf_documents(id INTEGER PRIMARY KEY AUTOINCREMENT, name TEXT NOT NULL, version INTEGER NOT NULL DEFAULT 1, xml TEXT NOT NULL, created_at INTEGER NOT NULL)",
                "CREATE TABLE IF NOT EXISTS urdf_versions(urdf_id INTEGER NOT NULL, version INTEGER NOT NULL, xml TEXT NOT NULL, applied_at INTEGER NOT NULL, PRIMARY KEY(urdf_id, version))",
                "CREATE TABLE IF NOT EXISTS calibration_sets(id INTEGER PRIMARY KEY AUTOINCREMENT, name TEXT NOT NULL, robot_serial TEXT, valid_from INTEGER, valid_to INTEGER, version INTEGER NOT NULL DEFAULT 1, created_at INTEGER NOT NULL)",
                "CREATE TABLE IF NOT EXISTS calibration_transforms(id INTEGER PRIMARY KEY AUTOINCREMENT, set_id INTEGER NOT NULL, parent TEXT NOT NULL, child TEXT NOT NULL, x REAL, y REAL, z REAL, roll REAL, pitch REAL, yaw REAL)",
                "CREATE TABLE IF NOT EXISTS snapshots(id INTEGER PRIMARY KEY AUTOINCREMENT, name TEXT NOT NULL, robot_serial TEXT, joint_values TEXT NOT NULL, captured_at INTEGER NOT NULL, calibration_id INTEGER)",
                "CREATE TABLE IF NOT EXISTS drafts(id INTEGER PRIMARY KEY AUTOINCREMENT, urdf_id INTEGER NOT NULL, base_version INTEGER NOT NULL, ops TEXT NOT NULL, status TEXT NOT NULL DEFAULT 'DRAFT', created_at INTEGER NOT NULL)"
            )) st.executeUpdate(sql)
        }
    }

    override fun close() = conn.close()

    // ---------- URDF documents ----------

    fun insertUrdf(name: String, xml: String): UrdfDoc {
        val now = System.currentTimeMillis()
        conn.prepareStatement("INSERT INTO urdf_documents(name, version, xml, created_at) VALUES(?,?,?,?)").use { ps ->
            ps.setString(1, name); ps.setInt(2, 1); ps.setString(3, xml); ps.setLong(4, now)
            ps.executeUpdate()
        }
        val id = lastId()
        conn.prepareStatement("INSERT INTO urdf_versions(urdf_id, version, xml, applied_at) VALUES(?,?,?,?)").use { ps ->
            ps.setLong(1, id); ps.setInt(2, 1); ps.setString(3, xml); ps.setLong(4, now)
            ps.executeUpdate()
        }
        return UrdfDoc(id, name, 1, xml, now)
    }

    fun getUrdf(id: Long): UrdfDoc? =
        conn.prepareStatement("SELECT id, name, version, xml, created_at FROM urdf_documents WHERE id=?").use { ps ->
            ps.setLong(1, id)
            val rs = ps.executeQuery()
            if (rs.next()) UrdfDoc(rs.getLong(1), rs.getString(2), rs.getInt(3), rs.getString(4), rs.getLong(5)) else null
        }

    fun listUrdf(): List<UrdfDoc> =
        conn.prepareStatement("SELECT id, name, version, xml, created_at FROM urdf_documents ORDER BY id").use { ps ->
            val rs = ps.executeQuery()
            buildList { while (rs.next()) add(UrdfDoc(rs.getLong(1), rs.getString(2), rs.getInt(3), rs.getString(4), rs.getLong(5))) }
        }

    fun urdfVersionXml(id: Long, version: Int): String? =
        conn.prepareStatement("SELECT xml FROM urdf_versions WHERE urdf_id=? AND version=?").use { ps ->
            ps.setLong(1, id); ps.setInt(2, version)
            val rs = ps.executeQuery()
            if (rs.next()) rs.getString(1) else null
        }

    /** Applies an approved draft with optimistic concurrency on the version number. */
    fun applyDraft(draftId: Long, newXml: String): Pair<Boolean, String> {
        conn.autoCommit = false
        try {
            val draft = getDraft(draftId) ?: return false to "draft not found"
            if (draft.status != "APPROVED") return false to "draft ${draft.id} is not APPROVED (status=${draft.status})"
            val doc = getUrdf(draft.urdfId) ?: return false to "urdf not found"
            if (doc.version != draft.baseVersion) {
                return false to "version conflict: base=${draft.baseVersion} current=${doc.version}"
            }
            val newVersion = doc.version + 1
            val now = System.currentTimeMillis()
            conn.prepareStatement("UPDATE urdf_documents SET version=?, xml=? WHERE id=? AND version=?").use { ps ->
                ps.setInt(1, newVersion); ps.setString(2, newXml)
                ps.setLong(3, doc.id); ps.setInt(4, doc.version)
                if (ps.executeUpdate() != 1) return false to "version conflict (concurrent update)"
            }
            conn.prepareStatement("INSERT INTO urdf_versions(urdf_id, version, xml, applied_at) VALUES(?,?,?,?)").use { ps ->
                ps.setLong(1, doc.id); ps.setInt(2, newVersion); ps.setString(3, newXml); ps.setLong(4, now)
                ps.executeUpdate()
            }
            conn.prepareStatement("UPDATE drafts SET status='APPLIED' WHERE id=?").use { ps ->
                ps.setLong(1, draftId); ps.executeUpdate()
            }
            conn.commit()
            return true to "applied as version $newVersion"
        } finally {
            conn.autoCommit = true
        }
    }

    // ---------- Calibration sets ----------

    fun insertCalibration(set: CalibrationSet): Long {
        conn.prepareStatement("INSERT INTO calibration_sets(name, robot_serial, valid_from, valid_to, version, created_at) VALUES(?,?,?,?,?,?)").use { ps ->
            ps.setString(1, set.name)
            if (set.robotSerial == null) ps.setNull(2, java.sql.Types.VARCHAR) else ps.setString(2, set.robotSerial)
            if (set.validFrom == null) ps.setNull(3, java.sql.Types.INTEGER) else ps.setLong(3, set.validFrom)
            if (set.validTo == null) ps.setNull(4, java.sql.Types.INTEGER) else ps.setLong(4, set.validTo)
            ps.setInt(5, set.version)
            ps.setLong(6, System.currentTimeMillis())
            ps.executeUpdate()
        }
        val id = lastId()
        for (t in set.transforms) {
            conn.prepareStatement("INSERT INTO calibration_transforms(set_id, parent, child, x, y, z, roll, pitch, yaw) VALUES(?,?,?,?,?,?,?,?,?)").use { ps ->
                ps.setLong(1, id); ps.setString(2, t.parent); ps.setString(3, t.child)
                ps.setDouble(4, t.xyz[0]); ps.setDouble(5, t.xyz[1]); ps.setDouble(6, t.xyz[2])
                ps.setDouble(7, t.rpy[0]); ps.setDouble(8, t.rpy[1]); ps.setDouble(9, t.rpy[2])
                ps.executeUpdate()
            }
        }
        return id
    }

    fun getCalibration(id: Long): CalibrationSet? = listCalibrations().firstOrNull { it.id == id }

    fun listCalibrations(): List<CalibrationSet> {
        val sets = conn.prepareStatement("SELECT id, name, robot_serial, valid_from, valid_to, version FROM calibration_sets ORDER BY id").use { ps ->
            val rs = ps.executeQuery()
            buildList {
                while (rs.next()) {
                    add(
                        CalibrationSet(
                            id = rs.getLong(1), name = rs.getString(2),
                            robotSerial = rs.getString(3),
                            validFrom = rs.getObject(4)?.let { (it as Number).toLong() },
                            validTo = rs.getObject(5)?.let { (it as Number).toLong() },
                            version = rs.getInt(6)
                        )
                    )
                }
            }
        }
        return sets.map { s ->
            val transforms = conn.prepareStatement("SELECT parent, child, x, y, z, roll, pitch, yaw FROM calibration_transforms WHERE set_id=? ORDER BY id").use { ps ->
                ps.setLong(1, s.id)
                val rs = ps.executeQuery()
                buildList {
                    while (rs.next()) {
                        add(
                            CalibrationTransform(
                                rs.getString(1), rs.getString(2),
                                listOf(rs.getDouble(3), rs.getDouble(4), rs.getDouble(5)),
                                listOf(rs.getDouble(6), rs.getDouble(7), rs.getDouble(8))
                            )
                        )
                    }
                }
            }
            s.copy(transforms = transforms)
        }
    }

    // ---------- Snapshots ----------

    fun insertSnapshot(s: Snapshot): Long {
        conn.prepareStatement("INSERT INTO snapshots(name, robot_serial, joint_values, captured_at, calibration_id) VALUES(?,?,?,?,?)").use { ps ->
            ps.setString(1, s.name)
            if (s.robotSerial == null) ps.setNull(2, java.sql.Types.VARCHAR) else ps.setString(2, s.robotSerial)
            ps.setString(3, json.encodeToString(s.jointValues))
            ps.setLong(4, s.capturedAt)
            if (s.calibrationId == null) ps.setNull(5, java.sql.Types.INTEGER) else ps.setLong(5, s.calibrationId)
            ps.executeUpdate()
        }
        return lastId()
    }

    fun getSnapshot(id: Long): Snapshot? = listSnapshots().firstOrNull { it.id == id }

    fun listSnapshots(): List<Snapshot> =
        conn.prepareStatement("SELECT id, name, robot_serial, joint_values, captured_at, calibration_id FROM snapshots ORDER BY id").use { ps ->
            val rs = ps.executeQuery()
            buildList {
                while (rs.next()) {
                    add(
                        Snapshot(
                            rs.getLong(1), rs.getString(2), rs.getString(3),
                            json.decodeFromString(rs.getString(4)),
                            rs.getLong(5), rs.getObject(6)?.let { (it as Number).toLong() }
                        )
                    )
                }
            }
        }

    // ---------- Drafts ----------

    fun insertDraft(urdfId: Long, baseVersion: Int, ops: List<PatchOp>): Long {
        conn.prepareStatement("INSERT INTO drafts(urdf_id, base_version, ops, status, created_at) VALUES(?,?,?,'DRAFT',?)").use { ps ->
            ps.setLong(1, urdfId); ps.setInt(2, baseVersion)
            ps.setString(3, json.encodeToString(ops))
            ps.setLong(4, System.currentTimeMillis())
            ps.executeUpdate()
        }
        return lastId()
    }

    fun getDraft(id: Long): Draft? =
        conn.prepareStatement("SELECT id, urdf_id, base_version, ops, status, created_at FROM drafts WHERE id=?").use { ps ->
            ps.setLong(1, id)
            val rs = ps.executeQuery()
            if (rs.next()) Draft(rs.getLong(1), rs.getLong(2), rs.getInt(3), json.decodeFromString(rs.getString(4)), rs.getString(5), rs.getLong(6)) else null
        }

    fun listDrafts(): List<Draft> =
        conn.prepareStatement("SELECT id, urdf_id, base_version, ops, status, created_at FROM drafts ORDER BY id").use { ps ->
            val rs = ps.executeQuery()
            buildList {
                while (rs.next()) add(Draft(rs.getLong(1), rs.getLong(2), rs.getInt(3), json.decodeFromString(rs.getString(4)), rs.getString(5), rs.getLong(6)))
            }
        }

    fun setDraftStatus(id: Long, status: String) {
        conn.prepareStatement("UPDATE drafts SET status=? WHERE id=?").use { ps ->
            ps.setString(1, status); ps.setLong(2, id); ps.executeUpdate()
        }
    }

    private fun lastId(): Long =
        conn.createStatement().use { st ->
            val rs = st.executeQuery("SELECT last_insert_rowid()")
            rs.next(); rs.getLong(1)
        }
}
