package jcb.kin

import jcb.model.JointSpec
import jcb.model.JointType

/**
 * 解析一次查询中每个活动关节的取值。
 *
 * 关节单位：
 *  - revolute / continuous：弧度（现场快照也以弧度存储，支持角度输入显式转换）
 *  - prismatic：米
 *
 * mimic 关系 q_this = multiplier * q_target + offset，成链时逐级复合；
 * mimic 成环时标记 MIMIC_CYCLE，绝不猜测数值。
 */
data class ResolvedJoint(
    val jointName: String,
    val value: Double?,
    val unit: JointUnit,
    val status: JointValueStatus,
    val detail: String,
    /** mimic 解析链：this -> target1 -> ... -> 提供原始值的关节。 */
    val mimicChain: List<String> = emptyList(),
)

enum class JointUnit { RADIANS, METERS, NONE }

enum class JointValueStatus {
    PROVIDED,
    MIMIC_DERIVED,
    MISSING,
    LIMIT_VIOLATION,
    MIMIC_CYCLE,
    MIMIC_TARGET_MISSING,
    UNRESOLVABLE_JOINT,
}

class JointResolver(
    private val jointsByName: Map<String, JointSpec>,
    /** 已统一为内部单位（弧度/米）的现场关节值。 */
    private val provided: Map<String, Double>,
) {
    private val memo = mutableMapOf<String, ResolvedJoint>()

    fun resolve(jointName: String): ResolvedJoint = memo.getOrPut(jointName) {
        compute(jointName, linkedSetOf())
    }

    private fun compute(name: String, stack: LinkedHashSet<String>): ResolvedJoint {
        val joint = jointsByName[name]
            ?: return ResolvedJoint(name, null, JointUnit.NONE,
                JointValueStatus.UNRESOLVABLE_JOINT, "关节在模型中不存在")
        if (joint.type == JointType.FIXED) {
            return ResolvedJoint(name, 0.0, JointUnit.NONE,
                JointValueStatus.PROVIDED, "固定关节，无自由度")
        }
        val unit = unitOf(joint.type)
        val mimic = joint.mimic
        if (mimic == null) {
            val raw = provided[name]
                ?: return ResolvedJoint(name, null, unit,
                    JointValueStatus.MISSING, "现场快照缺少关节值（${unitText(unit)}）")
            val limit = joint.limit
            if (joint.type == JointType.REVOLUTE && limit?.lower != null && limit.upper != null &&
                (raw < limit.lower || raw > limit.upper)
            ) {
                return ResolvedJoint(name, raw, unit, JointValueStatus.LIMIT_VIOLATION,
                    "关节值 $raw 超出限位 [${limit.lower}, ${limit.upper}]（弧度）")
            }
            return ResolvedJoint(name, raw, unit, JointValueStatus.PROVIDED,
                "现场快照提供（${unitText(unit)}）")
        }

        val chain = linkedSetOf<String>().also { it.addAll(stack); it.add(name) }
        val target = mimic.jointName
        if (!chain.add(target)) {
            return ResolvedJoint(name, null, unit, JointValueStatus.MIMIC_CYCLE,
                "mimic 关系在 $target 处形成环路，无法确定关节值", chain.toList())
        }
        val targetJoint = jointsByName[target]
        if (targetJoint == null) {
            return ResolvedJoint(name, null, unit, JointValueStatus.MIMIC_TARGET_MISSING,
                "mimic 目标关节 $target 不存在", chain.toList())
        }
        val targetUnit = unitOf(targetJoint.type)
        val base = memo[target] ?: compute(target, chain)
        val baseValue = base.value
        if (baseValue == null) {
            val status = when (base.status) {
                JointValueStatus.MISSING -> JointValueStatus.MIMIC_TARGET_MISSING
                else -> base.status
            }
            return ResolvedJoint(name, null, unit, status,
                "mimic 目标 $target 不可用：${base.detail}", chain.toList())
        }
        val value = mimic.multiplier * baseValue + mimic.offsetRadiansOrMeters
        return ResolvedJoint(name, value, unit, JointValueStatus.MIMIC_DERIVED,
            "q($name) = ${mimic.multiplier} × q($target) + ${mimic.offsetRadiansOrMeters}" +
                " = $value（${unitText(unit)}；目标单位 ${unitText(targetUnit)}）",
            chain.toList())
    }

    private fun unitOf(type: JointType) = when (type) {
        JointType.REVOLUTE, JointType.CONTINUOUS -> JointUnit.RADIANS
        JointType.PRISMATIC -> JointUnit.METERS
        else -> JointUnit.NONE
    }

    companion object {
        fun unitText(unit: JointUnit): String = when (unit) {
            JointUnit.RADIANS -> "弧度"
            JointUnit.METERS -> "米"
            JointUnit.NONE -> "无量纲"
        }
    }
}
