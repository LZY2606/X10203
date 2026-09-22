package jcb.db

import java.sql.Connection
import java.sql.DriverManager
import java.sql.ResultSet
import java.time.Instant

class Database(private val connection: Connection) {

    fun migrate() {
        connection.createStatement().use { st ->
            st.executeUpdate(
                """
                CREATE TABLE IF NOT EXISTS robots (
                    serial TEXT PRIMARY KEY,
                    name TEXT NOT NULL,
                    created_at TEXT NOT NULL
                )
                """.trimIndent()
            )
            st.executeUpdate(
                """
                CREATE TABLE IF NOT EXISTS urdf_versions (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    robot_serial TEXT NOT NULL REFERENCES robots(serial),
                    version INTEGER NOT NULL,
                    original_xml TEXT NOT NULL,
                    imported_at TEXT NOT NULL,
                    UNIQUE(robot_serial, version)
                )
                """.trimIndent()
            )
            st.executeUpdate(
                """
                CREATE TABLE IF NOT EXISTS calibration_versions (
                    id TEXT PRIMARY KEY,
                    robot_serial TEXT NOT NULL REFERENCES robots(serial),
                    version INTEGER NOT NULL,
                    label TEXT NOT NULL,
                    valid_from TEXT,
                    valid_until TEXT,
                    note TEXT NOT NULL DEFAULT '',
                    created_at TEXT NOT NULL,
                    UNIQUE(robot_serial, version)
                )
                """.trimIndent()
            )
            st.executeUpdate(
                """
                CREATE TABLE IF NOT EXISTS calibration_declarations (
                    id TEXT PRIMARY KEY,
                    calibration_id TEXT NOT NULL REFERENCES calibration_versions(id),
                    parent_frame TEXT NOT NULL,
                    child_frame TEXT NOT NULL,
                    xyz_meters TEXT NOT NULL,
                    rpy_radians TEXT NOT NULL,
                    ordinal INTEGER NOT NULL,
                    comment TEXT NOT NULL DEFAULT ''
                )
                """.trimIndent()
            )
            st.executeUpdate(
                """
                CREATE TABLE IF NOT EXISTS snapshots (
                    id TEXT PRIMARY KEY,
                    robot_serial TEXT NOT NULL REFERENCES robots(serial),
                    frozen_at TEXT NOT NULL,
                    values_json TEXT NOT NULL,
                    note TEXT NOT NULL DEFAULT '',
                    base_calibration_id TEXT
                )
                """.trimIndent()
            )
            st.executeUpdate(
                """
                CREATE TABLE IF NOT EXISTS urdf_patches (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    robot_serial TEXT NOT NULL,
                    urdf_version INTEGER NOT NULL,
                    target_type TEXT NOT NULL,
                    target_name TEXT NOT NULL,
                    operation TEXT NOT NULL,
                    payload TEXT NOT NULL,
                    status TEXT NOT NULL DEFAULT 'PROPOSED',
                    base_revision INTEGER NOT NULL,
                    created_at TEXT NOT NULL,
                    decided_at TEXT
                )
                """.trimIndent()
            )
            st.executeUpdate(
                """
                CREATE TABLE IF NOT EXISTS urdf_revisions (
                    robot_serial TEXT PRIMARY KEY,
                    current_revision INTEGER NOT NULL
                )
                """.trimIndent()
            )
        }
    }

    fun <T> tx(block: (Connection) -> T): T {
        val prev = connection.autoCommit
        connection.autoCommit = false
        try {
            val result = block(connection)
            connection.commit()
            return result
        } catch (t: Throwable) {
            connection.rollback()
            throw t
        } finally {
            connection.autoCommit = prev
        }
    }

    companion object {
        fun open(path: String): Database {
            Class.forName("org.sqlite.JDBC")
            val conn = DriverManager.getConnection("jdbc:sqlite:$path")
            conn.createStatement().use {
                it.execute("PRAGMA foreign_keys = ON")
                it.execute("PRAGMA journal_mode = WAL")
            }
            return Database(conn)
        }

        fun ResultSet.instant(column: String): Instant? =
            getString(column)?.let { Instant.parse(it) }
    }
}
