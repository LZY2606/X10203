package jcb.urdf

import jcb.db.PatchOperation
import jcb.db.PatchRow
import jcb.db.PatchTargetType
import jcb.xml.XmlAttribute
import jcb.xml.XmlCData
import jcb.xml.XmlDocument
import jcb.xml.XmlElement
import jcb.xml.XmlParser
import jcb.xml.XmlText
import kotlinx.serialization.json.Json
import kotlinx.serialization.json.JsonObject
import kotlinx.serialization.json.jsonPrimitive

/**
 * XML 导出严格只应用已批准补丁；未识别元素/属性/注释/CDATA/顺序全部保留。
 * 标定变换从不写入导出的 URDF（独立版本化）。
 */
class PatchExporter {
    private val json = Json { ignoreUnknownKeys = true }

    fun export(originalXml: String, approvedPatches: List<PatchRow>): String {
        val doc = XmlParser.parse(originalXml)
        val root = doc.root()
        approvedPatches
            .sortedWith(compareBy({ it.baseRevision }, { it.id }))
            .forEach { applyPatch(root, it) }
        return doc.serialize()
    }

    private fun applyPatch(root: XmlElement, patch: PatchRow) {
        val payload = json.parseToJsonElement(patch.payload) as JsonObject
        when (patch.operation) {
            PatchOperation.SET_ORIGIN -> {
                val target = findJoint(root, patch.targetName)
                val originEl = target.firstChild("origin")
                    ?: XmlElement("origin").also { el ->
                        it.selfClosing = true
                        insertOriginAfterParentChild(target, el)
                    }
                payload["xyz"]?.jsonPrimitive?.content?.let { originEl.setAttr("xyz", it) }
                payload["rpy"]?.jsonPrimitive?.content?.let { originEl.setAttr("rpy", it) }
            }
            PatchOperation.SET_LIMIT -> {
                val target = findJoint(root, patch.targetName)
                val limitEl = target.firstChild("limit")
                    ?: XmlElement("limit").also { it.selfClosing = true; target.children.add(it) }
                listOf("lower", "upper", "effort", "velocity").forEach { key ->
                    payload[key]?.jsonPrimitive?.content?.let { limitEl.setAttr(key, it) }
                }
            }
            PatchOperation.SET_AXIS -> {
                val target = findJoint(root, patch.targetName)
                val axisEl = target.firstChild("axis")
                    ?: XmlElement("axis").also { it.selfClosing = true; target.children.add(it) }
                payload["xyz"]?.jsonPrimitive?.content?.let { axisEl.setAttr("xyz", it) }
            }
            PatchOperation.SET_ATTRIBUTE -> {
                val target = findTargetElement(root, patch.targetType, patch.targetName)
                val name = payload["name"]!!.jsonPrimitive.content
                val value = payload["value"]!!.jsonPrimitive.content
                target.setAttr(name, value)
            }
            PatchOperation.ADD_ELEMENT -> {
                val target = findTargetElement(root, patch.targetType, patch.targetName)
                val tag = payload["tag"]!!.jsonPrimitive.content
                val element = XmlElement(tag)
                payload["attributes"]?.let { attrsRaw ->
                    (attrsRaw as JsonObject).forEach { (k, v) ->
                        element.setAttr(k, v.jsonPrimitive.content)
                    }
                }
                payload["text"]?.jsonPrimitive?.content?.let {
                    element.children.add(XmlText(it, it))
                }
                payload["cdata"]?.jsonPrimitive?.content?.let {
                    element.children.add(XmlCData(it))
                }
                if (element.children.isEmpty()) element.selfClosing = true
                target.children.add(element)
            }
            PatchOperation.REMOVE_ELEMENT -> {
                val target = findTargetElement(root, patch.targetType, patch.targetName)
                val tag = payload["tag"]!!.jsonPrimitive.content
                val name = payload["name"]?.jsonPrimitive?.content
                val victims = target.childElements(tag).filter { name == null || it.attr("name") == name }
                victims.forEach { target.children.remove(it) }
            }
        }
    }

    private fun insertOriginAfterParentChild(joint: XmlElement, origin: XmlElement) {
        val anchor = joint.childElements().indexOfFirst { it.tag == "parent" || it.tag == "child" }
        if (anchor >= 0) joint.children.add(anchor + 1, origin)
        else joint.children.add(0, origin)
    }

    private fun findJoint(root: XmlElement, name: String): XmlElement =
        root.childElements("joint").firstOrNull { it.attr("name") == name }
            ?: throw IllegalArgumentException("找不到 joint[@name=$name]，补丁无法应用")

    private fun findTargetElement(root: XmlElement, type: PatchTargetType, name: String): XmlElement = when (type) {
        PatchTargetType.ROBOT -> root
        PatchTargetType.JOINT -> findJoint(root, name)
        PatchTargetType.LINK -> root.childElements("link").firstOrNull { it.attr("name") == name }
            ?: throw IllegalArgumentException("找不到 link[@name=$name]，补丁无法应用")
    }

    /** 仅校验补丁能否在当前树上解析（不写回）。 */
    fun dryRun(originalXml: String, patch: PatchRow): String {
        val doc: XmlDocument = XmlParser.parse(originalXml)
        applyPatch(doc.root(), patch)
        return doc.serialize()
    }
}
