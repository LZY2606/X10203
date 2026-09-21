package coordbook

/** 边来源，用于在查询结果中说明姿态到底由哪些定义组成。 */
sealed interface EdgeSource {
    val label: String
    val kind: String
}

data class UrdfEdgeSource(val jointId: String, val jointName: String, val jointType: JointType) : EdgeSource {
    override val label = "urdf:joint:$jointName"
    override val kind = "urdf"
}

data class CalibEdgeSource(
    val calibVersionId: Long,
    val calibVersion: Int,
    val patchId: Long,
    val declaredBy: String,
) : EdgeSource {
    override val label = "calib-v$calibVersion:$declaredBy"
    override val kind = "calibration"
}

data class Frame(val id: String, val name: String, val origin: FrameOrigin) {
    enum class FrameOrigin { URDF_LINK }
}

/**
 * 有向多边图（multigraph）：同一对节点之间允许平行边——
 * URDF 同名重复关节以及标定声明的同一变换都各自保留，不按遍历顺序挑选。
 */
class FrameGraph {
    val frames = linkedMapOf<String, Frame>()
    val outgoing = linkedMapOf<String, MutableList<GraphEdge>>()

    fun addFrame(id: String, name: String) {
        frames.putIfAbsent(id, Frame(id, name, Frame.FrameOrigin.URDF_LINK))
        outgoing.putIfAbsent(id, mutableListOf())
    }

    fun addEdge(edge: GraphEdge) {
        addFrame(edge.from, edge.fromName)
        addFrame(edge.to, edge.toName)
        outgoing.getValue(edge.from).add(edge)
        outgoing.getValue(edge.to).add(edge.reversed())
    }

    fun neighbors(id: String): List<GraphEdge> = outgoing[id] ?: emptyList()

    fun frameIdsByName(name: String): List<String> =
        frames.values.filter { it.name == name }.map { it.id }

    /** 同一对 frame 之间由多个来源声明的同一变换（URDF 与标定重复声明）。 */
    fun parallelEdgeGroups(): List<List<GraphEdge>> {
        val groups = linkedMapOf<String, MutableList<GraphEdge>>()
        for (edges in outgoing.values) {
            for (e in edges) {
                if (e.reversedFlag) continue
                val key = setOf(e.from, e.to).sorted().joinToString("|")
                groups.getOrPut(key) { mutableListOf() }.add(e)
            }
        }
        return groups.values.filter { it.size > 1 }
    }
}

/**
 * 名义（零关节值）变换与关节轴，用于图上求值。
 * [nominal] 为 from -> to：T = origin * jointTransform(value)。
 */
data class GraphEdge(
    val from: String,
    val to: String,
    val fromName: String,
    val toName: String,
    val source: EdgeSource,
    val nominal: Transform,
    val jointType: JointType,
    val axisParentFrame: DoubleArray? = null,
    val requiredJoint: String? = null,
    val unit: String = "1",
    val reversedFlag: Boolean = false,
) {
    /** 在给定关节值下求 from->to 变换；缺失所需关节值返回 null。 */
    fun resolved(values: Map<String, Double>, resolver: JointValueResolver): EdgeEval {
        if (reversedFlag) error("内部错误：不应直接求值反向边")
        return when (source) {
            is CalibEdgeSource -> EdgeEval(nominal, null, null)
            is UrdfEdgeSource -> {
                val joint = resolver.joint(source.jointId)
                if (!joint.type.hasValue) return EdgeEval(nominal, null, null)
                when (val r = resolver.resolve(joint, values)) {
                    is JointValueResult.Ok -> {
                        // URDF：T_parent_child = origin * jointMotion(axis, value)，轴在关节 frame 内
                        val motion = if (joint.type == JointType.PRISMATIC) {
                            Transform.translation(
                                joint.axis[0] * r.value,
                                joint.axis[1] * r.value,
                                joint.axis[2] * r.value,
                            )
                        } else {
                            Transform.fromAxisAngle(joint.axis, r.value)
                        }
                        EdgeEval(joint.origin.transform() * motion, joint to r.value, r)
                    }
                    is JointValueResult.Missing -> EdgeEval(null, null, r)
                    is JointValueResult.Cycle -> EdgeEval(null, null, r)
                }
            }
        }
    }

    fun reversed(): GraphEdge = copy(
        from = to, to = from, fromName = toName, toName = fromName,
        nominal = nominal.inverse(), reversedFlag = true,
    )
}

data class EdgeEval(val transform: Transform?, val joint: Pair<JointNode, Double>?, val result: Any?)

sealed interface JointValueResult {
    data class Ok(val value: Double, val derivedFrom: String?, val unit: String) : JointValueResult
    data class Missing(val jointName: String, val reason: String) : JointValueResult
    data class Cycle(val jointName: String) : JointValueResult
}

class JointValueResolver(private val doc: UrdfDocument) {
    private val jointsByName = doc.joints.values.associateBy { it.name }

    fun joint(id: String): JointNode = doc.joints.getValue(id)

    fun resolve(joint: JointNode, values: Map<String, Double>, seen: MutableSet<String> = mutableSetOf()): JointValueResult {
        if (!seen.add(joint.name)) return JointValueResult.Cycle(joint.name)
        values[joint.name]?.let {
            return JointValueResult.Ok(it, null, joint.type.unit)
        }
        val mimic = joint.mimic ?: return JointValueResult.Missing(
            joint.name, "快照中缺少关节 \"${joint.name}\" 的值",
        )
        val parent = jointsByName[mimic.jointName]
            ?: return JointValueResult.Missing(
                joint.name,
                "mimic 目标 \"${mimic.jointName}\" 未定义",
            )
        return when (val base = resolve(parent, values, seen)) {
            is JointValueResult.Ok -> JointValueResult.Ok(
                mimic.multiplier * base.value + mimic.offset,
                parent.name,
                joint.type.unit,
            )
            is JointValueResult.Missing -> JointValueResult.Missing(
                joint.name,
                "mimic 链上 \"${parent.name}\" 缺值：${base.reason}",
            )
            is JointValueResult.Cycle -> JointValueResult.Cycle(joint.name)
        }
    }
}

data class ChainStep(
    val fromFrame: String,
    val toFrame: String,
    val edgeLabel: String,
    val edgeKind: String,
    val jointName: String?,
    val jointType: String?,
    val jointValue: Double?,
    val valueUnit: String?,
    val valueOrigin: String?,
    val transform: List<List<Double>>,
)

data class PathCandidate(
    val index: Int,
    val frames: List<String>,
    val steps: List<ChainStep>,
    val transform: List<List<Double>>,
)

data class MultiPathReport(
    val fromFrame: String,
    val toFrame: String,
    val candidates: List<PathCandidate>,
    val consistent: Boolean,
    val maxTranslationErrorM: Double,
    val maxRotationErrorRad: Double,
    val consensus: List<List<Double>>?,
    val missingJoint: MissingJointStatus?,
)

data class MissingJointStatus(val jointName: String, val reason: String)

object GraphBuilder {
    /** 构造只含 URDF 边的图；重名 link 导致的 parent/child 歧义作为问题返回。 */
    fun build(doc: UrdfDocument): Pair<FrameGraph, List<DocumentIssue>> {
        val graph = FrameGraph()
        val issues = mutableListOf<DocumentIssue>()
        for (link in doc.links.values) graph.addFrame(link.id, link.name)
        val names = doc.links.values.groupBy { it.name }
        for (joint in doc.joints.values) {
            val parents = (names[joint.parentLink] ?: emptyList()).map { it.id }
            val children = (names[joint.childLink] ?: emptyList()).map { it.id }
            if (parents.size > 1) {
                issues += DocumentIssue(
                    DocumentIssue.Severity.ERROR, "DUPLICATE_FRAME",
                    "joint \"${joint.name}\" 的 parent \"${joint.parentLink}\" 重名，无法唯一确定",
                    joint.id,
                )
            }
            if (children.size > 1) {
                issues += DocumentIssue(
                    DocumentIssue.Severity.ERROR, "DUPLICATE_FRAME",
                    "joint \"${joint.name}\" 的 child \"${joint.childLink}\" 重名，无法唯一确定",
                    joint.id,
                )
            }
            val from = parents.singleOrNull() ?: continue
            val to = children.singleOrNull() ?: continue
            graph.addEdge(
                GraphEdge(
                    from = from,
                    to = to,
                    fromName = joint.parentLink,
                    toName = joint.childLink,
                    source = UrdfEdgeSource(joint.id, joint.name, joint.type),
                    nominal = joint.origin.transform(),
                    jointType = joint.type,
                    requiredJoint = joint.name,
                    unit = joint.type.unit,
                ),
            )
        }
        // 重复声明的同一变换
        graph.parallelEdgeGroups().forEach { group ->
            issues += DocumentIssue(
                DocumentIssue.Severity.WARNING,
                "DUPLICATE_TRANSFORM",
                "frame 对 ${group.first().fromName} ↔ ${group.first().toName} 存在 ${group.size} 条平行声明：" +
                    group.joinToString { it.source.label },
            )
        }
        return graph to issues
    }
}
