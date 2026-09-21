package coordbook

/**
 * 标定补丁：独立于 URDF 版本化存储，绝不在写回时伪装成原 URDF。
 */
enum class PatchKind { EXTRA_TRANSFORM, JOINT_ORIGIN_RPY, JOINT_ORIGIN_XYZ }

enum class PatchStatus { PENDING, APPROVED, REJECTED }

data class CalibPatch(
    val id: Long,
    val versionId: Long,
    val kind: PatchKind,
    val targetJoint: String?,
    val parentFrame: String?,
    val childFrame: String?,
    val xyz: DoubleArray?,
    val rpyRadians: DoubleArray?,
    val status: PatchStatus,
    val note: String?,
    val createdAt: String,
)

data class CalibrationVersion(
    val id: Long,
    val robotId: Long,
    val serialScope: String,
    val validFromIso: String,
    val validUntilIso: String,
    val revision: Int,
    val note: String?,
    val createdAt: String,
)

/** 给定机器人序列号与查询时刻，标定是否生效。 */
enum class EffectiveState { EFFECTIVE, OUT_OF_WINDOW, SERIAL_MISMATCH, NO_VERSION }

data class EffectiveResult(val state: EffectiveState, val reason: String, val versionId: Long? = null)

object CalibrationService {

    /**
     * 时间窗为闭区间 [validFrom, validUntil]（端点生效），时间为 UTC ISO-8601。
     */
    fun effectiveAt(version: CalibrationVersion, serial: String, atIso: String): EffectiveResult {
        if (version.serialScope != serial && version.serialScope != "*") {
            return EffectiveResult(
                EffectiveState.SERIAL_MISMATCH,
                "标定 v${version.revision} 仅适用于序列号 \"${version.serialScope}\"，当前为 \"$serial\"",
                version.id,
            )
        }
        val at = IsoTime.parse(atIso)
        val from = IsoTime.parse(version.validFromIso)
        val until = IsoTime.parse(version.validUntilIso)
        return when {
            at.isBefore(from) -> EffectiveResult(
                EffectiveState.OUT_OF_WINDOW,
                "查询时刻 $atIso 早于生效起点 ${version.validFromIso}（边界起点生效，当前不生效）",
                version.id,
            )
            at.isAfter(until) -> EffectiveResult(
                EffectiveState.OUT_OF_WINDOW,
                "查询时刻 $atIso 晚于生效终点 ${version.validUntilIso}（边界终点生效，当前不生效）",
                version.id,
            )
            else -> EffectiveResult(
                EffectiveState.EFFECTIVE,
                "序列号与时间窗匹配，标定 v${version.revision} 在闭区间端点同样生效",
                version.id,
            )
        }
    }

    /** 选择某序列号/时刻生效的已审批版本（同窗口多个版本时取 revision 最大者）。 */
    fun selectEffective(
        versions: List<CalibrationVersion>,
        approval: (Long) -> Boolean,
        serial: String,
        atIso: String,
    ): Pair<CalibrationVersion?, EffectiveResult> {
        var chosen: CalibrationVersion? = null
        var states = versions.map { it to effectiveAt(it, serial, atIso) }
        states = states.filter { it.second.state == EffectiveState.EFFECTIVE && approval(it.first.id) }
        chosen = states.maxByOrNull { it.first.revision }?.first
        val result = when {
            chosen != null -> EffectiveResult(
                EffectiveState.EFFECTIVE,
                "命中标定 v${chosen.revision}（闭区间端点生效）", chosen.id,
            )
            versions.isEmpty() -> EffectiveResult(EffectiveState.NO_VERSION, "该机器人没有任何标定版本")
            else -> {
                val any = versions.first()
                val sample = effectiveAt(any, serial, atIso)
                if (sample.state == EffectiveState.SERIAL_MISMATCH &&
                    versions.all { effectiveAt(it, serial, atIso).state == EffectiveState.SERIAL_MISMATCH }
                ) {
                    EffectiveResult(EffectiveState.SERIAL_MISMATCH,
                        "没有适用于序列号 \"$serial\" 的标定版本")
                } else {
                    EffectiveResult(EffectiveState.OUT_OF_WINDOW,
                        "时刻 $atIso 不在任何已审批标定版本的生效闭区间内（含端点）")
                }
            }
        }
        return chosen to result
    }

    /**
     * 把已审批补丁叠加到 URDF 图上：
     *  - EXTRA_TRANSFORM 增加一条标定边（与 URDF 同变换声明并存为平行边）
     *  - JOINT_ORIGIN_* 不修改图结构，查询时替换对应关节 origin
     */
    fun applyApproved(
        graph: FrameGraph,
        patches: List<CalibPatch>,
        versionId: Long,
        version: Int,
    ): List<DocumentIssue> {
        val issues = mutableListOf<DocumentIssue>()
        for (patch in patches.filter {
            it.status == PatchStatus.APPROVED && it.versionId == versionId
        }) {
            when (patch.kind) {
                PatchKind.EXTRA_TRANSFORM -> {
                    val parent = patch.parentFrame ?: continue
                    val child = patch.childFrame ?: continue
                    val fromIds = graph.frameIdsByName(parent)
                    val toIds = graph.frameIdsByName(child)
                    if (fromIds.size != 1 || toIds.size != 1) {
                        issues += DocumentIssue(
                            DocumentIssue.Severity.WARNING,
                            "CALIB_FRAME_AMBIGUOUS",
                            "标定补丁 #${patch.id} 的 frame \"$parent\"/\"$child\" 无法唯一解析，已跳过",
                            "patch:${patch.id}",
                        )
                        continue
                    }
                    val t = Pose(patch.xyz ?: DoubleArray(3), patch.rpyRadians ?: DoubleArray(3)).transform()
                    graph.addEdge(
                        GraphEdge(
                            from = fromIds.single(),
                            to = toIds.single(),
                            fromName = parent,
                            toName = child,
                            source = CalibEdgeSource(
                                versionId, version, patch.id,
                                "extra:${parent}->${child}",
                            ),
                            nominal = t,
                            jointType = JointType.FIXED,
                            unit = "1",
                        ),
                    )
                }
                else -> Unit // origin 覆盖在 effectiveGraph 阶段处理
            }
        }
        for (patch in patches.filter {
            it.versionId == versionId &&
                it.kind in listOf(PatchKind.JOINT_ORIGIN_RPY, PatchKind.JOINT_ORIGIN_XYZ)
        }) {
            val jointName = patch.targetJoint ?: continue
            val edges = graph.frames.keys.flatMap { graph.neighbors(it) }
                .filter { !it.reversedFlag && (it.source as? UrdfEdgeSource)?.jointName == jointName }
            for (edge in edges) {
                val oldT = edge.nominal
                val xyz = patch.xyz ?: doubleArrayOf(oldT.m[3], oldT.m[7], oldT.m[11])
                // origin 覆盖只替换平移或重新给定的 rpy；JOINT_ORIGIN_XYZ 时保留原旋转
                val t = if (patch.kind == PatchKind.JOINT_ORIGIN_RPY) {
                    Pose(
                        doubleArrayOf(oldT.m[3], oldT.m[7], oldT.m[11]),
                        patch.rpyRadians ?: DoubleArray(3),
                    ).transform()
                } else {
                    Pose(xyz, DoubleArray(3)).transform().let { newT ->
                        // 保留原旋转部分
                        val merged = newT.m.copyOf()
                        for (r in 0 until 3) for (c in 0 until 3) merged[r * 4 + c] = oldT.m[r * 4 + c]
                        Transform(merged)
                    }
                }
                replaceNominal(graph, edge, t)
            }
        }
        // 重复声明（URDF 与标定）平行边告警
        graph.parallelEdgeGroups().forEach { group ->
            val kinds = group.map { it.source.kind }.toSet()
            if ("urdf" in kinds && "calibration" in kinds) {
                issues += DocumentIssue(
                    DocumentIssue.Severity.WARNING,
                    "URDF_CALIB_DUPLICATE",
                    "frame 对 ${group.first().fromName} ↔ ${group.first().toName} 的同一变换被 " +
                        "URDF 与标定同时声明，保留为 ${group.size} 条候选边并计算误差",
                )
            }
        }
        return issues
    }
}


private fun replaceNominal(graph: FrameGraph, edge: GraphEdge, t: Transform) {
    for (list in graph.outgoing.values) {
        for (i in list.indices) {
            val e = list[i]
            if (!e.reversedFlag && e.source == edge.source) list[i] = e.copy(nominal = t)
            if (e.reversedFlag && e.source == edge.source) list[i] = e.copy(nominal = t.inverse())
        }
    }
}
