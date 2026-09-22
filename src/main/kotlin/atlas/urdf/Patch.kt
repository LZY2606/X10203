package atlas.urdf

import atlas.store.PatchOp
import atlas.xml.XmlDocument
import atlas.xml.XmlElement
import kotlinx.serialization.Serializable

@Serializable
data class DiffEntry(
    val kind: String,       // joint-origin | joint-axis | joint-limit | joint-added | joint-removed | link-added | link-removed
    val name: String,
    val before: String? = null,
    val after: String? = null
)

object PatchApplier {
    /** Applies approved patch ops onto a deep copy of the document; unknown content is untouched. */
    fun apply(doc: XmlDocument, ops: List<PatchOp>): XmlDocument {
        val root = doc.root.deepCopy()
        for (op in ops) {
            val jointEl = root.childElements("joint").firstOrNull { it.attr("name") == op.joint }
                ?: throw IllegalArgumentException("joint not found: ${op.joint}")
            when (op.type) {
                "setOrigin" -> {
                    val origin = jointEl.firstChild("origin") ?: XmlElement("origin").also {
                        jointEl.children.add(0, it)
                    }
                    op.xyz?.let { origin.attributes["xyz"] = it.joinToString(" ") }
                    op.rpy?.let { origin.attributes["rpy"] = it.joinToString(" ") }
                }
                "setAxis" -> {
                    val axis = jointEl.firstChild("axis") ?: XmlElement("axis").also {
                        jointEl.children.add(it)
                    }
                    op.axis?.let { axis.attributes["xyz"] = it.joinToString(" ") }
                }
                "setLimit" -> {
                    val limit = jointEl.firstChild("limit") ?: XmlElement("limit").also {
                        jointEl.children.add(it)
                    }
                    op.lower?.let { limit.attributes["lower"] = it.toString() }
                    op.upper?.let { limit.attributes["upper"] = it.toString() }
                }
                else -> throw IllegalArgumentException("unknown patch op: ${op.type}")
            }
        }
        return XmlDocument(doc.prolog, root)
    }
}

object UrdfDiff {
    fun diff(beforeXml: String, afterXml: String): List<DiffEntry> {
        val a = UrdfParser.parse(beforeXml)
        val b = UrdfParser.parse(afterXml)
        val out = mutableListOf<DiffEntry>()
        val aJoints = a.joints.associateBy { it.name }
        val bJoints = b.joints.associateBy { it.name }
        for ((name, aj) in aJoints) {
            val bj = bJoints[name]
            if (bj == null) {
                out.add(DiffEntry("joint-removed", name, aj.type.tag, null))
                continue
            }
            if (aj.origin != bj.origin) {
                out.add(DiffEntry("joint-origin", name, originStr(aj.origin), originStr(bj.origin)))
            }
            if (aj.axis != bj.axis) {
                out.add(DiffEntry("joint-axis", name, aj.axis?.toString(), bj.axis?.toString()))
            }
            if (aj.limit != bj.limit) {
                out.add(DiffEntry("joint-limit", name, aj.limit?.toString(), bj.limit?.toString()))
            }
        }
        for ((name, bj) in bJoints) {
            if (name !in aJoints) out.add(DiffEntry("joint-added", name, null, bj.type.tag))
        }
        val aLinks = a.links.map { it.name }.toSet()
        val bLinks = b.links.map { it.name }.toSet()
        for (l in aLinks - bLinks) out.add(DiffEntry("link-removed", l))
        for (l in bLinks - aLinks) out.add(DiffEntry("link-added", l))
        return out
    }

    private fun originStr(o: Origin): String =
        "xyz=${o.xyz.x},${o.xyz.y},${o.xyz.z} rpy=${o.rpy.roll},${o.rpy.pitch},${o.rpy.yaw}"
}
