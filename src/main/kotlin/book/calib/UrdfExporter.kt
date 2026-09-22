package book.calib

import book.urdf.UrdfModel
import book.xml.RawDocument
import book.xml.RawElement
import book.xml.RawText

/**
 * 导出器：把“获批补丁”应用到 URDF 原始文档的副本上。
 *  - 只应用 approved 补丁中列出的 edge
 *  - 未识别元素/属性/注释原样保留（直接复用 RawDocument 副本）
 *  - 输出必须带派生声明，绝不伪装成原始 URDF
 */
object UrdfExporter {

    data class ExportResult(
        val xml: String,
        val appliedEdges: List<String>,
        val skippedUnapproved: List<String>,
        val skippedMissing: List<String>,
        val warnings: List<String>,
    )

    fun applyApproved(
        model: UrdfModel,
        calibration: CalibrationVersion,
        approvedEdgeIds: Set<String>,
    ): ExportResult {
        val doc: RawDocument = model.let {
            // 深拷贝：通过渲染+重解析获得结构副本（未知内容仍保留在文本中）
            book.xml.RawXmlParser(it.document.render()).parse()
        }
        val root = doc.root!!
        val applied = mutableListOf<String>()
        val skippedUnapproved = mutableListOf<String>()
        val skippedMissing = mutableListOf<String>()
        val warnings = mutableListOf<String>()

        calibration.edges.forEach { edge ->
            if (edge.id !in approvedEdgeIds) {
                skippedUnapproved += edge.id
                return@forEach
            }
            when (edge.kind) {
                "override" -> {
                    val jointName = edge.jointName
                    val jointEl = root.directElements()
                        .firstOrNull { it.name == "joint" && it.attr("name") == jointName }
                    if (jointEl == null) {
                        skippedMissing += edge.id
                        warnings += "补丁 ${edge.id}：joint '$jointName' 在 URDF 中不存在，已跳过"
                        return@forEach
                    }
                    var originEl = jointEl.firstChild("origin")
                    if (originEl == null) {
                        originEl = RawElement("origin")
                        // 插到 parent/child 之后，保持 URDF 常见顺序
                        val insertAt = jointEl.children.indexOfLast {
                            (it as? RawElement)?.name in setOf("parent", "child")
                        } + 1
                        jointEl.children.add(insertAt.coerceAtLeast(0), originEl)
                    }
                    originEl.setAttr("xyz", edge.xyz.joinToString(" ") { fmt(it) })
                    originEl.setAttr("rpy", edge.rpyRad.joinToString(" ") { fmt(it) })
                    originEl.attributes.add(book.xml.RawAttr("calib:edge", edge.id))
                    applied += edge.id
                }
                "new" -> {
                    val jointName = "calib_${edge.id.replace('-', '_')}"
                    val jointEl = RawElement("joint", mutableListOf(
                        book.xml.RawAttr("name", jointName),
                        book.xml.RawAttr("type", "fixed"),
                    ))
                    jointEl.children.add(RawText("\n"))
                    val origin = RawElement("origin", mutableListOf(
                        book.xml.RawAttr("xyz", edge.xyz.joinToString(" ") { fmt(it) }),
                        book.xml.RawAttr("rpy", edge.rpyRad.joinToString(" ") { fmt(it) }),
                        book.xml.RawAttr("calib:edge", edge.id),
                    ))
                    val parent = RawElement("parent",
                        mutableListOf(book.xml.RawAttr("link", edge.parentFrame)))
                    val child = RawElement("child",
                        mutableListOf(book.xml.RawAttr("link", edge.childFrame)))
                    jointEl.children.add(RawText("  "))
                    jointEl.children.add(parent)
                    jointEl.children.add(RawText("\n  "))
                    jointEl.children.add(child)
                    jointEl.children.add(RawText("\n  "))
                    jointEl.children.add(origin)
                    jointEl.children.add(RawText("\n"))
                    root.children.add(RawText("  "))
                    root.children.add(jointEl)
                    root.children.add(RawText("\n"))
                    applied += edge.id
                }
                else -> {
                    skippedMissing += edge.id
                    warnings += "补丁 ${edge.id}：未知类型 ${edge.kind}，已跳过"
                }
            }
        }

        val stamp = buildString {
            append("\n<!-- ===========================================================\n")
            append("  本文件由「关节坐标册」基于 URDF 与标定派生，并非原始 URDF！\n")
            append("  标定版本: v${calibration.version} (${calibration.id})\n")
            append("  机器人序列号: ${calibration.robotSerial}\n")
            append("  生效窗口: [${calibration.validFrom}, ${calibration.validUntil})\n")
            append("  应用补丁: ${if (applied.isEmpty()) "无" else applied.joinToString()}\n")
            append("  未批准而跳过: ${if (skippedUnapproved.isEmpty()) "无" else skippedUnapproved.joinToString()}\n")
            append("  原始未知元素/属性均已保留；请勿将本文件写回为原始 URDF。\n")
            append("=========================================================== -->\n")
        }
        doc.prolog.add(RawText(stamp))

        return ExportResult(doc.render(), applied, skippedUnapproved, skippedMissing, warnings)
    }

    private fun fmt(v: Double): String {
        if (v == 0.0) return "0"
        val s = "%.9f".format(v).trimEnd('0').trimEnd('.')
        return s
    }
}
