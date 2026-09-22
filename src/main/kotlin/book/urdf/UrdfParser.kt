package book.urdf

import book.math.Vec3
import book.xml.RawElement
import book.xml.RawXmlParser

/** 解析 URDF；未知元素/属性留在 RawDocument 中，不删除也不报错，仅在需要处登记 issue。 */
object UrdfParser {

    fun parse(text: String): UrdfModel {
        val doc = RawXmlParser(text).parse()
        val issues = mutableListOf<ParseIssue>()
        val root = doc.root
        if (root == null || root.name != "robot") {
            issues += ParseIssue("error", "NO_ROBOT_ROOT", "缺少 <robot> 根元素")
            return UrdfModel(null, emptyList(), emptyList(), doc, issues)
        }

        val links = mutableListOf<Link>()
        val joints = mutableListOf<Joint>()

        root.directElements().forEach { el ->
            when (el.name) {
                "link" -> {
                    val name = el.attr("name")
                    if (name.isNullOrBlank()) {
                        issues += ParseIssue("error", "LINK_NO_NAME", "存在缺少 name 的 <link>")
                    } else {
                        if (links.any { it.name == name }) {
                            issues += ParseIssue("warning", "DUPLICATE_LINK",
                                "link '$name' 被重复定义（第 ${links.size + 1} 次出现）")
                        }
                        links += parseLink(name, links.size, el)
                    }
                }
                "joint" -> {
                    val name = el.attr("name")
                    val typeName = el.attr("type")
                    if (name.isNullOrBlank()) {
                        issues += ParseIssue("error", "JOINT_NO_NAME", "存在缺少 name 的 <joint>")
                        return@forEach
                    }
                    if (joints.any { it.name == name }) {
                        issues += ParseIssue("warning", "DUPLICATE_JOINT",
                            "joint '$name' 被重复定义")
                    }
                    val parent = el.firstChild("parent")?.attr("link")
                    val child = el.firstChild("child")?.attr("link")
                    if (parent == null || child == null) {
                        issues += ParseIssue("error", "JOINT_ENDPOINTS",
                            "joint '$name' 缺少 parent/child")
                    }
                    if (typeName == null) {
                        issues += ParseIssue("warning", "JOINT_NO_TYPE", "joint '$name' 缺少 type，按 unknown 处理")
                    }
                    joints += parseJoint(name, joints.size, el, issues)
                }
                else -> { /* 未知顶层元素：保留在 document 中，登记但不报错 */
                    issues += ParseIssue("info", "UNKNOWN_ELEMENT",
                        "保留未识别的顶层元素 <${el.name}>（name=${el.attr("name") ?: "-"}）")
                }
            }
        }

        val linkNames = links.map { it.name }.toSet()
        joints.forEach { j ->
            if (j.parent !in linkNames) {
                issues += ParseIssue("warning", "DANGLING_PARENT",
                    "joint '${j.name}' 的 parent link '${j.parent}' 未定义")
            }
            if (j.child !in linkNames) {
                issues += ParseIssue("warning", "DANGLING_CHILD",
                    "joint '${j.name}' 的 child link '${j.child}' 未定义")
            }
        }
        detectMimicCycles(joints, issues)

        return UrdfModel(root.attr("name"), links, joints, doc, issues)
    }

    private fun parseLink(name: String, index: Int, el: RawElement): Link {
        val inertial = el.firstChild("inertial")?.let { ie ->
            val mass = ie.firstChild("mass")?.attr("value")?.toDoubleOrNull()
            val inertia = ie.firstChild("inertia")?.let { ine ->
                val a = arrayOf(
                    doubleArrayOf(
                        ine.attr("ixx")?.toDoubleOrNull() ?: 0.0,
                        ine.attr("ixy")?.toDoubleOrNull() ?: 0.0,
                        ine.attr("ixz")?.toDoubleOrNull() ?: 0.0,
                    ),
                    doubleArrayOf(
                        ine.attr("ixy")?.toDoubleOrNull() ?: 0.0,
                        ine.attr("iyy")?.toDoubleOrNull() ?: 0.0,
                        ine.attr("iyz")?.toDoubleOrNull() ?: 0.0,
                    ),
                    doubleArrayOf(
                        ine.attr("ixz")?.toDoubleOrNull() ?: 0.0,
                        ine.attr("iyz")?.toDoubleOrNull() ?: 0.0,
                        ine.attr("izz")?.toDoubleOrNull() ?: 0.0,
                    ),
                )
                a
            }
            InertialSpec(PoseSpec.parse(ie.firstChild("origin")), mass, inertia)
        }
        fun geometries(tag: String) = el.childElements(tag).mapNotNull { it.firstChild("geometry") }
            .map { g -> parseGeometry(g) }
        return Link(name, index, inertial, geometries("visual"), geometries("collision"), el)
    }

    private fun parseGeometry(g: RawElement): GeometrySpec {
        val inner = g.directElements().firstOrNull()
            ?: return GeometrySpec("unknown", emptyMap(), g)
        val detail = inner.attributes.associate { it.name to it.value }
        return GeometrySpec(inner.name, detail, g)
    }

    private fun parseJoint(name: String, index: Int, el: RawElement, issues: MutableList<ParseIssue>): Joint {
        val parent = el.firstChild("parent")?.attr("link") ?: ""
        val child = el.firstChild("child")?.attr("link") ?: ""
        val type = JointType.parse(el.attr("type"))
        val axis = parseVec(el.firstChild("axis")?.attr("xyz"), Vec3(1.0, 0.0, 0.0))
        val mimicEl = el.firstChild("mimic")
        val mimic = mimicEl?.let {
            val target = it.attr("joint")
            if (target == null) {
                issues += ParseIssue("error", "MIMIC_NO_TARGET", "joint '$name' 的 mimic 缺少 joint 属性")
                null
            } else {
                MimicSpec(
                    target,
                    it.attr("multiplier")?.toDoubleOrNull() ?: 1.0,
                    it.attr("offset")?.toDoubleOrNull() ?: 0.0,
                )
            }
        }
        val limitEl = el.firstChild("limit")
        val limit = if (limitEl != null) {
            JointLimit(
                limitEl.attr("lower")?.toDoubleOrNull(),
                limitEl.attr("upper")?.toDoubleOrNull(),
                limitEl.attr("effort")?.toDoubleOrNull(),
                limitEl.attr("velocity")?.toDoubleOrNull(),
                true,
            )
        } else {
            JointLimit(null, null, null, null, false)
        }
        return Joint(
            name, index, type, parent, child,
            PoseSpec.parse(el.firstChild("origin")),
            axis, mimic, limit, el,
        )
    }

    /** mimic 引用图中的环：q = m*q_other + o 形成环时无法唯一求解。 */
    private fun detectMimicCycles(joints: List<Joint>, issues: MutableList<ParseIssue>) {
        val byName = joints.associateBy { it.name }
        fun dfs(start: String) {
            val state = HashMap<String, Int>() // 0=访问中 1=完成
            fun visit(jn: String, path: List<String>): Boolean {
                state[jn] = 0
                val next = byName[jn]?.mimic?.jointName
                if (next != null && byName.containsKey(next)) {
                    when (state[next]) {
                        0 -> {
                            issues += ParseIssue("error", "MIMIC_CYCLE",
                                "mimic 环: ${(path.drop(path.indexOf(next)) + next).joinToString(" -> ")}")
                            return true
                        }
                        null -> if (visit(next, path + next)) return true
                    }
                }
                state[jn] = 1
                return false
            }
            visit(start, listOf(start))
        }
        joints.forEach { dfs(it.name) }
    }
}
