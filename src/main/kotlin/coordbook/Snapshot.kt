package coordbook

data class Snapshot(
    val id: Long,
    val robotId: Long,
    val serial: String,
    val frozenAtIso: String,
    val jointValues: Map<String, Double>, // revolute/continuous 单位 rad，prismatic 单位 m
    val note: String?,
    val createdAt: String,
)

data class EffectiveGraph(
    val graph: FrameGraph,
    val calibVersion: CalibrationVersion?,
    val effective: EffectiveResult,
    val appliedPatchIds: List<Long>,
    val issues: List<DocumentIssue>,
)

object SnapshotService {

    /**
     * 基于冻结快照 + 指定标定版本构建查询用图。
     * 标定不生效时仅使用 URDF 图，并在状态中明确原因。
     */
    fun buildEffectiveGraph(
        doc: UrdfDocument,
        versions: List<CalibrationVersion>,
        patches: List<CalibPatch>,
        serial: String,
        atIso: String,
    ): EffectiveGraph {
        val (graph, issues) = GraphBuilder.build(doc)
        val (version, result) = CalibrationService.selectEffective(
            versions,
            approval = { vid -> patches.any { it.versionId == vid && it.status == PatchStatus.APPROVED } },
            serial,
            atIso,
        )
        if (version == null) {
            return EffectiveGraph(graph, null, result, emptyList(), issues)
        }
        val approved = patches.filter { it.versionId == version.id && it.status == PatchStatus.APPROVED }
        val calibIssues = CalibrationService.applyApproved(graph, approved, version.id, version.revision)
        return EffectiveGraph(
            graph, version, result, approved.map { it.id }, issues + calibIssues,
        )
    }
}
