package atlas

data class Limit(val lower: Double?, val upper: Double?, val effort: Double?, val velocity: Double?)

data class Mimic(val joint: String, val multiplier: Double, val offset: Double)

data class Inertial(val mass: Double?, val xyz: DoubleArray, val rpy: DoubleArray)

data class Geometry(val type: String, val params: Map<String, String>)

data class UrdfJoint(
    val name: String,
    val type: String,
    val parent: String,
    val child: String,
    val xyz: DoubleArray,
    val rpy: DoubleArray,
    val axis: DoubleArray,
    val limit: Limit?,
    val mimic: Mimic?
)

data class UrdfLink(val name: String, val inertial: Inertial?, val geometries: List<Geometry>)

data class UrdfModel(
    val robotName: String,
    val links: List<UrdfLink>,
    val joints: List<UrdfJoint>,
    val duplicateFrames: List<String>,
    val doc: XElement
)

fun parseDoubles(s: String?, def: DoubleArray): DoubleArray {
    if (s == null) return def
    val parts = s.trim().split(Regex("\\s+")).mapNotNull { it.toDoubleOrNull() }
    return if (parts.size == def.size) parts.toDoubleArray() else def
}

fun parseUrdf(doc: XElement): UrdfModel {
    val robot = Xml.document(doc)
    val links = robot.children("link").map { le ->
        val inertial = le.child("inertial")?.let { ie ->
            Inertial(
                ie.child("mass")?.attr("value")?.toDoubleOrNull(),
                parseDoubles(ie.child("origin")?.attr("xyz"), doubleArrayOf(0.0, 0.0, 0.0)),
                parseDoubles(ie.child("origin")?.attr("rpy"), doubleArrayOf(0.0, 0.0, 0.0))
            )
        }
        val geoms = le.children("visual").mapNotNull { it.child("geometry") }.map { ge ->
            val g = ge.children.filterIsInstance<XElement>().firstOrNull()
            Geometry(g?.name ?: "unknown", g?.attrs ?: linkedMapOf())
        }
        UrdfLink(le.attr("name") ?: "", inertial, geoms)
    }
    val joints = robot.children("joint").map { je ->
        val origin = je.child("origin")
        val lim = je.child("limit")
        val mim = je.child("mimic")
        UrdfJoint(
            je.attr("name") ?: "",
            je.attr("type") ?: "fixed",
            je.child("parent")?.attr("link") ?: "",
            je.child("child")?.attr("link") ?: "",
            parseDoubles(origin?.attr("xyz"), doubleArrayOf(0.0, 0.0, 0.0)),
            parseDoubles(origin?.attr("rpy"), doubleArrayOf(0.0, 0.0, 0.0)),
            parseDoubles(je.child("axis")?.attr("xyz"), doubleArrayOf(1.0, 0.0, 0.0)),
            lim?.let {
                Limit(
                    it.attr("lower")?.toDoubleOrNull(),
                    it.attr("upper")?.toDoubleOrNull(),
                    it.attr("effort")?.toDoubleOrNull(),
                    it.attr("velocity")?.toDoubleOrNull()
                )
            },
            mim?.let {
                Mimic(
                    it.attr("joint") ?: "",
                    it.attr("multiplier")?.toDoubleOrNull() ?: 1.0,
                    it.attr("offset")?.toDoubleOrNull() ?: 0.0
                )
            }
        )
    }
    val names = links.map { it.name }
    val dups = names.groupingBy { it }.eachCount().filter { it.value > 1 }.keys.toList()
    return UrdfModel(robot.attr("name") ?: "robot", links, joints, dups, doc)
}
