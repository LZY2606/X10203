package coordbook

import kotlinx.serialization.json.Json
import kotlinx.serialization.json.JsonArray
import kotlinx.serialization.json.JsonObject
import kotlinx.serialization.json.double
import kotlinx.serialization.json.jsonArray
import kotlinx.serialization.json.jsonObject
import kotlinx.serialization.json.jsonPrimitive
import org.w3c.dom.Document
import org.w3c.dom.Element

/** Approved-only URDF patches, applied to a parsed DOM at export time. Unknown content is untouched. */
object Patches {
    fun applyAll(doc: Document, approvedPatchJsons: List<String>) {
        for (pj in approvedPatchJsons) {
            val ops = Json.parseToJsonElement(pj).jsonObject["ops"]?.jsonArray ?: continue
            for (opEl in ops) applyOp(doc, opEl.jsonObject)
        }
    }

    private fun applyOp(doc: Document, op: JsonObject) {
        when (op["op"]?.jsonPrimitive?.content) {
            "set_joint_origin" -> {
                val je = jointElement(doc, op["joint"]!!.jsonPrimitive.content) ?: return
                val origin = je.childElements("origin").firstOrNull()
                    ?: doc.createElement("origin").also { je.insertBefore(it, je.firstChild) }
                op["xyz"]?.jsonArray?.let { origin.setAttribute("xyz", it.join()) }
                op["rpy"]?.jsonArray?.let { origin.setAttribute("rpy", it.join()) }
            }
            "set_joint_axis" -> {
                val je = jointElement(doc, op["joint"]!!.jsonPrimitive.content) ?: return
                val axis = je.childElements("axis").firstOrNull()
                    ?: doc.createElement("axis").also { je.appendChild(it) }
                op["xyz"]?.jsonArray?.let { axis.setAttribute("xyz", it.join()) }
            }
            "set_limit" -> {
                val je = jointElement(doc, op["joint"]!!.jsonPrimitive.content) ?: return
                val limit = je.childElements("limit").firstOrNull()
                    ?: doc.createElement("limit").also { je.appendChild(it) }
                op["lower"]?.jsonPrimitive?.content?.let { limit.setAttribute("lower", it) }
                op["upper"]?.jsonPrimitive?.content?.let { limit.setAttribute("upper", it) }
            }
            // unknown ops are ignored, never silently dropped from the draft record
        }
    }

    private fun jointElement(doc: Document, name: String): Element? =
        doc.documentElement.childElements("joint").firstOrNull { it.getAttribute("name") == name }

    private fun JsonArray.join() = joinToString(" ") { it.jsonPrimitive.content }

    /** Structural diff between two URDF versions (joints/origins/limits/links). */
    fun diff(a: UrdfRobot, b: UrdfRobot): List<String> {
        val out = mutableListOf<String>()
        val aJ = a.joints.associateBy { it.name }; val bJ = b.joints.associateBy { it.name }
        for (n in aJ.keys - bJ.keys) out.add("joint removed: $n")
        for (n in bJ.keys - aJ.keys) out.add("joint added: $n")
        for (n in aJ.keys intersect bJ.keys) {
            val x = aJ[n]!!; val y = bJ[n]!!
            if (!x.origin.xyz.contentEquals(y.origin.xyz) || !x.origin.rpy.contentEquals(y.origin.rpy))
                out.add("joint $n origin: ${fmt(x.origin)} -> ${fmt(y.origin)}")
            if (!x.axis.contentEquals(y.axis)) out.add("joint $n axis changed")
            if (x.limit != y.limit) out.add("joint $n limit: ${x.limit} -> ${y.limit}")
            if (x.mimic != y.mimic) out.add("joint $n mimic: ${x.mimic} -> ${y.mimic}")
        }
        val aL = a.links.map { it.name }.toSet(); val bL = b.links.map { it.name }.toSet()
        for (n in aL - bL) out.add("link removed: $n")
        for (n in bL - aL) out.add("link added: $n")
        return out
    }

    private fun fmt(o: Origin) = "xyz=${o.xyz.toList()} rpy=${o.rpy.toList()}"
}
