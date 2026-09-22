package atlas

import java.sql.Connection
import java.sql.DriverManager

data class UrdfVersion(val id: Int, val name: String, val xml: String, val createdAt: Long)

data class Draft(
    val id: Int,
    val baseVersion: Int,
    val description: String,
    val patch: String,
    val status: String,
    val createdAt: Long
)

data class Snapshot(val id: Int, val name: String, val joints: String, val note: String, val createdAt: Long)

class Store(path: String) {
    val conn: Connection = DriverManager.getConnection("jdbc:sqlite:$path")

    init {
        conn.createStatement().use { st ->
            st.executeUpdate(
                """CREATE TABLE IF NOT EXISTS urdf_versions(
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    name TEXT NOT NULL,
                    xml TEXT NOT NULL,
                    created_at INTEGER NOT NULL)"""
            )
            st.executeUpdate(
                """CREATE TABLE IF NOT EXISTS calibs(
                    rowid INTEGER PRIMARY KEY AUTOINCREMENT,
                    id TEXT NOT NULL,
                    version INTEGER NOT NULL,
                    from_frame TEXT NOT NULL,
                    to_frame TEXT NOT NULL,
                    xyz TEXT NOT NULL,
                    rpy TEXT NOT NULL,
                    unit TEXT NOT NULL,
                    serial TEXT NOT NULL,
                    valid_from INTEGER NOT NULL,
                    valid_to INTEGER NOT NULL,
                    created_at INTEGER NOT NULL)"""
            )
            st.executeUpdate(
                """CREATE TABLE IF NOT EXISTS drafts(
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    base_version INTEGER NOT NULL,
                    description TEXT NOT NULL,
                    patch TEXT NOT NULL,
                    status TEXT NOT NULL,
                    created_at INTEGER NOT NULL)"""
            )
            st.executeUpdate(
                """CREATE TABLE IF NOT EXISTS snapshots(
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    name TEXT NOT NULL,
                    joints TEXT NOT NULL,
                    note TEXT NOT NULL,
                    created_at INTEGER NOT NULL)"""
            )
        }
    }

    fun currentVersion(): Int = conn.createStatement().use { st ->
        st.executeQuery("SELECT COALESCE(MAX(id), 0) FROM urdf_versions").use { rs ->
            if (rs.next()) rs.getInt(1) else 0
        }
    }

    fun addUrdf(name: String, xml: String): Int {
        conn.prepareStatement("INSERT INTO urdf_versions(name, xml, created_at) VALUES(?,?,?)").use { ps ->
            ps.setString(1, name)
            ps.setString(2, xml)
            ps.setLong(3, System.currentTimeMillis() / 1000)
            ps.executeUpdate()
        }
        return currentVersion()
    }

    fun getUrdf(version: Int?): UrdfVersion? {
        val sql = if (version == null) {
            "SELECT id, name, xml, created_at FROM urdf_versions ORDER BY id DESC LIMIT 1"
        } else {
            "SELECT id, name, xml, created_at FROM urdf_versions WHERE id = $version"
        }
        conn.createStatement().use { st ->
            st.executeQuery(sql).use { rs ->
                if (!rs.next()) return null
                return UrdfVersion(rs.getInt(1), rs.getString(2), rs.getString(3), rs.getLong(4))
            }
        }
    }

    fun listVersions(): List<UrdfVersion> {
        val out = mutableListOf<UrdfVersion>()
        conn.createStatement().use { st ->
            st.executeQuery("SELECT id, name, xml, created_at FROM urdf_versions ORDER BY id").use { rs ->
                while (rs.next()) out.add(UrdfVersion(rs.getInt(1), rs.getString(2), rs.getString(3), rs.getLong(4)))
            }
        }
        return out
    }

    fun addCalib(c: CalibTransform) {
        conn.prepareStatement(
            """INSERT INTO calibs(id, version, from_frame, to_frame, xyz, rpy, unit, serial, valid_from, valid_to, created_at)
               VALUES(?,?,?,?,?,?,?,?,?,?,?)"""
        ).use { ps ->
            ps.setString(1, c.id)
            ps.setInt(2, c.version)
            ps.setString(3, c.from)
            ps.setString(4, c.to)
            ps.setString(5, c.xyz.joinToString(" "))
            ps.setString(6, c.rpy.joinToString(" "))
            ps.setString(7, c.rpyUnit)
            ps.setString(8, c.serial)
            ps.setLong(9, c.validFrom)
            ps.setLong(10, c.validTo)
            ps.setLong(11, System.currentTimeMillis() / 1000)
            ps.executeUpdate()
        }
    }

    fun calibs(): List<CalibTransform> {
        val out = mutableListOf<CalibTransform>()
        conn.createStatement().use { st ->
            st.executeQuery("SELECT id, version, from_frame, to_frame, xyz, rpy, unit, serial, valid_from, valid_to FROM calibs ORDER BY id, version").use { rs ->
                while (rs.next()) {
                    out.add(
                        CalibTransform(
                            rs.getString(1), rs.getInt(2), rs.getString(3), rs.getString(4),
                            parseDoubles(rs.getString(5), doubleArrayOf(0.0, 0.0, 0.0)),
                            parseDoubles(rs.getString(6), doubleArrayOf(0.0, 0.0, 0.0)),
                            rs.getString(7), rs.getString(8), rs.getLong(9), rs.getLong(10)
                        )
                    )
                }
            }
        }
        return out
    }

    fun nextCalibVersion(id: String): Int {
        conn.prepareStatement("SELECT COALESCE(MAX(version), 0) FROM calibs WHERE id = ?").use { ps ->
            ps.setString(1, id)
            ps.executeQuery().use { rs -> return (if (rs.next()) rs.getInt(1) else 0) + 1 }
        }
    }

    fun createDraft(baseVersion: Int, description: String, patch: String): Int {
        conn.prepareStatement("INSERT INTO drafts(base_version, description, patch, status, created_at) VALUES(?,?,?,'PENDING',?)").use { ps ->
            ps.setInt(1, baseVersion)
            ps.setString(2, description)
            ps.setString(3, patch)
            ps.setLong(4, System.currentTimeMillis() / 1000)
            ps.executeUpdate()
        }
        conn.createStatement().use { st ->
            st.executeQuery("SELECT MAX(id) FROM drafts").use { rs ->
                rs.next()
                return rs.getInt(1)
            }
        }
    }

    fun getDraft(id: Int): Draft? {
        conn.prepareStatement("SELECT id, base_version, description, patch, status, created_at FROM drafts WHERE id = ?").use { ps ->
            ps.setInt(1, id)
            ps.executeQuery().use { rs ->
                if (!rs.next()) return null
                return Draft(rs.getInt(1), rs.getInt(2), rs.getString(3), rs.getString(4), rs.getString(5), rs.getLong(6))
            }
        }
    }

    fun listDrafts(): List<Draft> {
        val out = mutableListOf<Draft>()
        conn.createStatement().use { st ->
            st.executeQuery("SELECT id, base_version, description, patch, status, created_at FROM drafts ORDER BY id").use { rs ->
                while (rs.next()) {
                    out.add(Draft(rs.getInt(1), rs.getInt(2), rs.getString(3), rs.getString(4), rs.getString(5), rs.getLong(6)))
                }
            }
        }
        return out
    }

    fun setDraftStatus(id: Int, status: String) {
        conn.prepareStatement("UPDATE drafts SET status = ? WHERE id = ?").use { ps ->
            ps.setString(1, status)
            ps.setInt(2, id)
            ps.executeUpdate()
        }
    }

    fun addSnapshot(name: String, joints: String, note: String): Int {
        conn.prepareStatement("INSERT INTO snapshots(name, joints, note, created_at) VALUES(?,?,?,?)").use { ps ->
            ps.setString(1, name)
            ps.setString(2, joints)
            ps.setString(3, note)
            ps.setLong(4, System.currentTimeMillis() / 1000)
            ps.executeUpdate()
        }
        conn.createStatement().use { st ->
            st.executeQuery("SELECT MAX(id) FROM snapshots").use { rs ->
                rs.next()
                return rs.getInt(1)
            }
        }
    }

    fun snapshots(): List<Snapshot> {
        val out = mutableListOf<Snapshot>()
        conn.createStatement().use { st ->
            st.executeQuery("SELECT id, name, joints, note, created_at FROM snapshots ORDER BY id").use { rs ->
                while (rs.next()) out.add(Snapshot(rs.getInt(1), rs.getString(2), rs.getString(3), rs.getString(4), rs.getLong(5)))
            }
        }
        return out
    }
}
