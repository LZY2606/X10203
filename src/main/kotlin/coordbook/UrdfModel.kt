package coordbook

/** 文档级问题：重名、断链、mimic 环、重复变换声明等。 */
data class DocumentIssue(
    val severity: Severity,
    val code: String,
    val message: String,
    val refId: String? = null,
) {
    enum class Severity { ERROR, WARNING, INFO }
}

data class Pose(val xyz: DoubleArray, val rpyRadians: DoubleArray) {
    fun transform(): Transform {
        val t = Transform.fromRpy(rpyRadians[0], rpyRadians[1], rpyRadians[2])
        t.m[3] = xyz[0]; t.m[7] = xyz[1]; t.m[11] = xyz[2]
        return t
    }

    companion object {
        val ORIGIN = Pose(DoubleArray(3), DoubleArray(3))
    }
}

data class Inertial(
    val massKg: Double,
    val origin: Pose,
    val ixx: Double, val ixy: Double, val ixz: Double,
    val iyy: Double, val iyz: Double, val izz: Double,
)

data class Geometry(
    val kind: Kind,
    val params: Map<String, String>,
) {
    enum class Kind { BOX, CYLINDER, SPHERE, MESH }
}

data class VisualCollision(val origin: Pose, val geometry: Geometry)

data class LinkNode(
    val id: String,
    val name: String,
    val ordinal: Int,
    val inertial: Inertial?,
    val visuals: List<VisualCollision>,
    val collisions: List<VisualCollision>,
    val element: XmlElement,
)

data class JointLimit(
    val lowerRadOrM: Double?,
    val upperRadOrM: Double?,
    val effort: Double?,
    val velocity: Double?,
)

data class MimicSpec(val jointName: String, val multiplier: Double, val offset: Double)

enum class JointType(val urdfName: String, val unit: String) {
    REVOLUTE("revolute", "rad"),
    CONTINUOUS("continuous", "rad"),
    PRISMATIC("prismatic", "m"),
    FIXED("fixed", "1"),
    FLOATING("floating", "mixed"),
    PLANAR("planar", "mixed");

    val hasValue: Boolean get() = this == REVOLUTE || this == CONTINUOUS || this == PRISMATIC
}

data class JointNode(
    val id: String,
    val name: String,
    val ordinal: Int,
    val type: JointType,
    val parentLink: String,
    val childLink: String,
    val origin: Pose,
    val axis: DoubleArray,
    val limit: JointLimit?,
    val mimic: MimicSpec?,
    val element: XmlElement,
)

data class UrdfDocument(
    val root: XmlElement,
    val xmlDeclaration: String?,
    val links: Map<String, LinkNode>,
    val joints: Map<String, JointNode>,
    val issues: List<DocumentIssue>,
) {
    val linkOrder: List<String> get() = links.values.sortedBy { it.ordinal }.map { it.id }
    val jointOrder: List<String> get() = joints.values.sortedBy { it.ordinal }.map { it.id }
}

object UrdfParser {

    private val JOINT_TYPES = JointType.entries.associateBy { it.urdfName }

    fun parse(xml: String): UrdfDocument {
        val (root, declaration) = XmlParser.parse(xml)
        if (root.tag != "robot") {
            throw XmlException("根元素应为 <robot>，实际为 <${root.tag}>")
        }
        val issues = mutableListOf<DocumentIssue>()
        val links = linkedMapOf<String, LinkNode>()
        val joints = linkedMapOf<String, JointNode>()

        root.allElements().forEach { child ->
            when (child.tag) {
                "link" -> {
                    val name = child.requireAttr("name")
                    val ordinal = links.size
                    val id = if (links.containsKey("link::$name") ||
                        joints.containsKey("joint::$name")
                    ) {
                        issues += DocumentIssue(
                            DocumentIssue.Severity.ERROR,
                            "DUPLICATE_FRAME",
                            "link 名称重复：$name（第 $ordinal 个 link，查询该名称将产生歧义）",
                        )
                        "link::$name#$ordinal"
                    } else "link::$name"
                    links[id] = parseLink(id, name, ordinal, child)
                }
                "joint" -> {
                    val name = child.requireAttr("name")
                    val ordinal = joints.size
                    val id = if (joints.containsKey("joint::$name") ||
                        links.containsKey("link::$name")
                    ) {
                        issues += DocumentIssue(
                            DocumentIssue.Severity.ERROR,
                            "DUPLICATE_FRAME",
                            "joint 名称与既有 frame 冲突：$name",
                        )
                        "joint::$name#$ordinal"
                    } else "joint::$name"
                    joints[id] = parseJoint(id, name, ordinal, child)
                }
                else -> issues += DocumentIssue(
                    DocumentIssue.Severity.INFO,
                    "UNKNOWN_ELEMENT",
                    "保留未识别的顶层元素 <${child.tag}>",
                )
            }
        }

        validate(links, joints, issues)
        return UrdfDocument(root, declaration, links, joints, issues)
    }

    private fun parseLink(id: String, name: String, ordinal: Int, el: XmlElement): LinkNode {
        val inertial = el.child("inertial")?.let(::parseInertial)
        val visuals = el.children("visual").map { VisualCollision(parsePoseIn(it), parseGeometryIn(it)) }
        val collisions = el.children("collision").map {
            VisualCollision(parsePoseIn(it), parseGeometryIn(it))
        }
        return LinkNode(id, name, ordinal, inertial, visuals, collisions, el)
    }

    private fun parseInertial(el: XmlElement): Inertial {
        val mass = el.child("mass")?.doubleAttr("value") ?: 0.0
        val origin = el.child("origin")?.let(::parsePose) ?: Pose.ORIGIN
        val inertia = el.child("inertia")
        return Inertial(
            mass, origin,
            inertia?.doubleAttr("ixx") ?: 0.0,
            inertia?.doubleAttr("ixy") ?: 0.0,
            inertia?.doubleAttr("ixz") ?: 0.0,
            inertia?.doubleAttr("iyy") ?: 0.0,
            inertia?.doubleAttr("iyz") ?: 0.0,
            inertia?.doubleAttr("izz") ?: 0.0,
        )
    }

    private fun parseGeometryIn(parent: XmlElement): Geometry {
        val geo = parent.child("geometry")
            ?: throw XmlException("visual/collision 缺少 <geometry>")
        val shape = geo.allElements().firstOrNull()
            ?: throw XmlException("<geometry> 内缺少几何形状元素")
        val params = shape.attributes.associate { it.first to XmlParser.unescape(it.second) }
        val kind = when (shape.tag) {
            "box" -> Geometry.Kind.BOX
            "cylinder" -> Geometry.Kind.CYLINDER
            "sphere" -> Geometry.Kind.SPHERE
            "mesh" -> Geometry.Kind.MESH
            else -> throw XmlException("不支持的几何形状 <${shape.tag}>（内容仍会原样保留）")
        }
        return Geometry(kind, params)
    }

    private fun parsePoseIn(parent: XmlElement): Pose =
        parent.child("origin")?.let(::parsePose) ?: Pose.ORIGIN

    private fun parsePose(el: XmlElement): Pose =
        Pose(parseTriplet(el.attr("xyz")), parseTriplet(el.attr("rpy")))

    private fun parseJoint(id: String, name: String, ordinal: Int, el: XmlElement): JointNode {
        val typeName = el.requireAttr("type")
        val type = JOINT_TYPES[typeName]
            ?: throw XmlException("joint \"$name\" 的 type=\"$typeName\" 不受支持")
        val parent = el.child("parent")?.requireAttr("link")
            ?: throw XmlException("joint \"$name\" 缺少 <parent link=...>")
        val child = el.child("child")?.requireAttr("link")
            ?: throw XmlException("joint \"$name\" 缺少 <child link=...>")
        val origin = el.child("origin")?.let(::parsePose) ?: Pose.ORIGIN
        val axis = el.child("axis")?.let { parseTriplet(it.attr("xyz")) }
            ?: doubleArrayOf(1.0, 0.0, 0.0)
        val limit = el.child("limit")?.let {
            JointLimit(
                it.doubleAttr("lower"),
                it.doubleAttr("upper"),
                it.doubleAttr("effort"),
                it.doubleAttr("velocity"),
            )
        }
        val mimic = el.child("mimic")?.let {
            MimicSpec(
                it.requireAttr("joint"),
                it.doubleAttr("multiplier") ?: 1.0,
                it.doubleAttr("offset") ?: 0.0,
            )
        }
        return JointNode(id, name, ordinal, type, parent, child, origin, axis, limit, mimic, el)
    }

    private fun validate(
        links: Map<String, LinkNode>,
        joints: Map<String, JointNode>,
        issues: MutableList<DocumentIssue>,
    ) {
        // 断链：joint 引用的 link 不存在
        for (joint in joints.values) {
            if (links.none { it.value.name == joint.parentLink }) {
                issues += DocumentIssue(
                    DocumentIssue.Severity.ERROR,
                    "BROKEN_CHAIN",
                    "joint \"${joint.name}\" 的 parent link \"${joint.parentLink}\" 未定义",
                    joint.id,
                )
            }
            if (links.none { it.value.name == joint.childLink }) {
                issues += DocumentIssue(
                    DocumentIssue.Severity.ERROR,
                    "BROKEN_CHAIN",
                    "joint \"${joint.name}\" 的 child link \"${joint.childLink}\" 未定义",
                    joint.id,
                )
            }
        }
        // mimic 环
        val byName = joints.values.associateBy { it.name }
        for (joint in joints.values) {
            if (joint.mimic == null) continue
            if (byName[joint.mimic.jointName] == null) {
                issues += DocumentIssue(
                    DocumentIssue.Severity.ERROR,
                    "MIMIC_BROKEN",
                    "joint \"${joint.name}\" mimic 的 \"${joint.mimic.jointName}\" 不存在",
                    joint.id,
                )
                continue
            }
            if (detectMimicCycle(joint, byName)) {
                issues += DocumentIssue(
                    DocumentIssue.Severity.ERROR,
                    "MIMIC_CYCLE",
                    "mimic 依赖经过 \"${joint.name}\" 形成环，无法展开为树",
                    joint.id,
                )
            }
        }
        // 同名关节定义重复（相同 parent/child 的多条边）在图构建阶段再报
    }

    private fun detectMimicCycle(start: JointNode, byName: Map<String, JointNode>): Boolean {
        var current = start
        val guard = mutableSetOf(start.name)
        while (current.mimic != null) {
            val next = byName[current.mimic!!.jointName] ?: return false
            if (next.name == start.name) return true
            if (!guard.add(next.name)) return false
            current = next
        }
        return false
    }
}
