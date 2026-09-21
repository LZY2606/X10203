package coordbook

/**
 * 在保真 XML DOM 上应用已审批补丁。
 * 只触碰已知 URDF 关节的 <origin> 属性；未知元素/属性/顺序/注释全部原样保留。
 * 标定额外变换（EXTRA_TRANSFORM）不写进 URDF，避免“标定假装是原 URDF”。
 */
object XmlPatcher {

    data class AppliedPatch(val patchId: Long, val jointName: String, val changed: List<String>)

    fun applyApprovedPatches(
        doc: UrdfDocument,
        patches: List<CalibPatch>,
    ): Pair<String, List<AppliedPatch>> {
        val applied = mutableListOf<AppliedPatch>()
        for (patch in patches.filter { it.status == PatchStatus.APPROVED }.sortedBy { it.id }) {
            when (patch.kind) {
                PatchKind.JOINT_ORIGIN_RPY, PatchKind.JOINT_ORIGIN_XYZ -> {
                    val jointName = patch.targetJoint ?: continue
                    val joint = doc.joints.values.firstOrNull { it.name == jointName } ?: continue
                    val origin = joint.element.child("origin") ?: continue
                    val changed = mutableListOf<String>()
                    if (patch.kind == PatchKind.JOINT_ORIGIN_RPY && patch.rpyRadians != null) {
                        origin.setAttr("rpy", patch.rpyRadians.fmt())
                        changed += "rpy"
                    }
                    if (patch.kind == PatchKind.JOINT_ORIGIN_XYZ && patch.xyz != null) {
                        origin.setAttr("xyz", patch.xyz.fmt())
                        changed += "xyz"
                    }
                    if (changed.isNotEmpty()) applied += AppliedPatch(patch.id, jointName, changed)
                }
                PatchKind.EXTRA_TRANSFORM -> Unit // 刻意不写入 URDF
            }
        }
        return doc.root.toXmlString(doc.xmlDeclaration) to applied
    }

    /** 原始 URDF 原样导出（往返保真校验用）。 */
    fun exportRaw(doc: UrdfDocument): String = doc.root.toXmlString(doc.xmlDeclaration)
}

/**
 * 标定独立导出：自有文档格式，顶部声明数据来源不是原 URDF。
 */
object CalibExporter {
    fun export(
        version: CalibrationVersion,
        patches: List<CalibPatch>,
    ): String = buildString {
        appendLine("<?xml version=\"1.0\" encoding=\"UTF-8\"?>")
        appendLine("<!-- 标定独立版本（calibration），不是原始 URDF；序列号与时间窗外不生效 -->")
        appendLine("<calibration version=\"${version.revision}\" robotSerial=\"${version.serialScope}\"")
        appendLine("             validFrom=\"${version.validFromIso}\" validUntil=\"${version.validUntilIso}\">")
        for (patch in patches.sortedBy { it.id }) {
            append("  <patch id=\"${patch.id}\" status=\"${patch.status.name.lowercase()}\" kind=\"${patch.kind.name.lowercase()}\"")
            if (patch.targetJoint != null) append(" joint=\"${patch.targetJoint}\"")
            appendLine(">")
            if (patch.parentFrame != null) {
                appendLine("    <frames parent=\"${patch.parentFrame}\" child=\"${patch.childFrame}\"/>")
            }
            if (patch.xyz != null) appendLine("    <xyz>${patch.xyz.fmt()}</xyz>")
            if (patch.rpyRadians != null) appendLine("    <!-- rpy 单位：弧度 --><rpy>${patch.rpyRadians.fmt()}</rpy>")
            if (patch.note != null) appendLine("    <note>${XmlParser.escapeRaw(patch.note, false)}</note>")
            appendLine("  </patch>")
        }
        appendLine("</calibration>")
    }
}

/** 两个标定版本之间的补丁差异。 */
data class VersionDiff(
    val baseRevision: Int,
    val targetRevision: Int,
    val added: List<CalibPatch>,
    val removed: List<CalibPatch>,
    val changedStatus: List<StatusChange>,
) {
    data class StatusChange(val patchId: Long, val from: PatchStatus, val to: PatchStatus, val kind: PatchKind)
}

object VersionDiffer {
    fun diff(base: List<CalibPatch>, target: List<CalibPatch>, br: Int, tr: Int): VersionDiff {
        val baseById = base.associateBy { it.id }
        val targetById = target.associateBy { it.id }
        val added = target.filter { it.id !in baseById }
        val removed = base.filter { it.id !in targetById }
        val changed = target.mapNotNull { t ->
            val b = baseById[t.id] ?: return@mapNotNull null
            if (b.status != t.status) {
                VersionDiff.StatusChange(t.id, b.status, t.status, t.kind)
            } else null
        }
        return VersionDiff(br, tr, added, removed, changed)
    }
}
