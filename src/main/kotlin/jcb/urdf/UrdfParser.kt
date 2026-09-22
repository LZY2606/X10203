package jcb.urdf

import jcb.model.AxisSpec
import jcb.model.GeometrySpec
import jcb.model.InertialSpec
import jcb.model.JointSpec
import jcb.model.JointType
import jcb.model.LimitSpec
import jcb.model.LinkSpec
import jcb.model.MimicSpec
import jcb.model.OriginSpec
import jcb.model.UrdfModel
import jcb.model.Vec3Spec
import jcb.model.VisualCollisionSpec
import jcb.xml.XmlElement
import jcb.xml.XmlParser

object UrdfParser {

    fun parse(xmlText: String): UrdfModel {
        val doc = XmlParser.parse(xmlText)
        val root = doc.root()
        require(root.tag == "robot") { "根元素必须是 <robot>，实际为 <${root.tag}>" }
        val warnings = mutableListOf<String>()

        val linkCounts = mutableMapOf<String, Int>()
        val links = root.childElements("link").map { el ->
            val name = el.attr("name") ?: run {
                warnings.add("存在缺少 name 属性的 <link>")
                "__unnamed_link_${linkCounts.size}"
            }
            val idx = linkCounts.merge(name, 0) { old, _ -> old + 1 }!!
            if (idx > 0) warnings.add("link 名称重复：$name（第 ${idx + 1} 次声明）")
            parseLink(name, el, idx, warnings)
        }

        val jointCounts = mutableMapOf<String, Int>()
        val joints = root.childElements("joint").map { el ->
            val name = el.attr("name") ?: run {
                warnings.add("存在缺少 name 属性的 <joint>")
                "__unnamed_joint_${jointCounts.size}"
            }
            val idx = jointCounts.merge(name, 0) { old, _ -> old + 1 }!!
            if (idx > 0) warnings.add("joint 名称重复：$name（第 ${idx + 1} 次声明）")
            parseJoint(name, el, idx, warnings)
        }

        return UrdfModel(
            robotName = root.attr("name"),
            links = links,
            joints = joints,
            rootElement = root,
            parseWarnings = warnings,
        )
    }

    private fun parseLink(name: String, el: XmlElement, idx: Int, warnings: MutableList<String>): LinkSpec {
        val inertialEl = el.firstChild("inertial")
        var inertial: InertialSpec? = null
        if (inertialEl != null) {
            val mass = inertialEl.firstChild("mass")?.attr("value")?.toDoubleOrNull()
            if (inertialEl.firstChild("mass") != null && mass == null) {
                warnings.add("link $name 的 inertial/mass 数值无法解析")
            }
            val inertia = inertialEl.firstChild("inertia")
            inertial = InertialSpec(
                massKg = mass,
                ixx = inertia?.attr("ixx")?.toDoubleOrNull() ?: 0.0,
                ixy = inertia?.attr("ixy")?.toDoubleOrNull() ?: 0.0,
                ixz = inertia?.attr("ixz")?.toDoubleOrNull() ?: 0.0,
                iyy = inertia?.attr("iyy")?.toDoubleOrNull() ?: 0.0,
                iyz = inertia?.attr("iyz")?.toDoubleOrNull() ?: 0.0,
                izz = inertia?.attr("izz")?.toDoubleOrNull() ?: 0.0,
            )
        }
        val visuals = el.childElements("visual").map { parseVisual(it, "visual", name, warnings) }
        val collisions = el.childElements("collision").map { parseVisual(it, "collision", name, warnings) }
        return LinkSpec(name, inertial, visuals, collisions, el, idx)
    }

    private fun parseVisual(el: XmlElement, kind: String, linkName: String, warnings: MutableList<String>): VisualCollisionSpec {
        val originEl = el.firstChild("origin")
        val origin = originEl?.let { parseOrigin(it, "$kind of $linkName", warnings) }
        val geometryEl = el.firstChild("geometry")
        var geometry: GeometrySpec? = null
        if (geometryEl != null) {
            val shapeEl = geometryEl.childElements().firstOrNull()
            if (shapeEl == null) {
                warnings.add("link $linkName 的 $kind/geometry 缺少形状元素")
            } else {
                geometry = when (shapeEl.tag) {
                    "box" -> GeometrySpec.Box(parseVec(shapeEl.attr("size")) ?: Vec3Spec(0.0, 0.0, 0.0))
                    "cylinder" -> GeometrySpec.Cylinder(
                        shapeEl.attr("radius")?.toDoubleOrNull() ?: 0.0,
                        shapeEl.attr("length")?.toDoubleOrNull() ?: 0.0,
                    )
                    "sphere" -> GeometrySpec.Sphere(shapeEl.attr("radius")?.toDoubleOrNull() ?: 0.0)
                    "mesh" -> GeometrySpec.Mesh(
                        shapeEl.attr("filename") ?: "",
                        parseVec(shapeEl.attr("scale")),
                    )
                    else -> {
                        warnings.add("link $linkName 的几何形状 <${shapeEl.tag}> 未识别，仍保留原文")
                        null
                    }
                }
            }
        }
        return VisualCollisionSpec(el.attr("name"), origin, geometry, el)
    }

    private fun parseJoint(name: String, el: XmlElement, idx: Int, warnings: MutableList<String>): JointSpec {
        val type = when (el.attr("type")) {
            "revolute" -> JointType.REVOLUTE
            "continuous" -> JointType.CONTINUOUS
            "prismatic" -> JointType.PRISMATIC
            "fixed" -> JointType.FIXED
            "floating" -> JointType.FLOATING
            "planar" -> JointType.PLANAR
            null -> {
                warnings.add("joint $name 缺少 type 属性")
                JointType.UNKNOWN
            }
            else -> {
                warnings.add("joint $name 的类型 ${el.attr("type")} 未识别")
                JointType.UNKNOWN
            }
        }
        val parent = el.firstChild("parent")?.attr("link")
        val child = el.firstChild("child")?.attr("link")
        if (parent == null) warnings.add("joint $name 缺少 <parent link=...>")
        if (child == null) warnings.add("joint $name 缺少 <child link=...>")
        val originEl = el.firstChild("origin")
        val origin = originEl?.let { parseOrigin(it, "joint $name", warnings) }
        val axisEl = el.firstChild("axis")
        val axis = parseVec(axisEl?.attr("xyz"))?.let { AxisSpec(it.x, it.y, it.z) } ?: AxisSpec.DEFAULT
        val limitEl = el.firstChild("limit")
        var limit: LimitSpec? = null
        if (limitEl != null) {
            limit = LimitSpec(
                lower = limitEl.attr("lower")?.toDoubleOrNull(),
                upper = limitEl.attr("upper")?.toDoubleOrNull(),
                effort = limitEl.attr("effort")?.toDoubleOrNull(),
                velocity = limitEl.attr("velocity")?.toDoubleOrNull(),
            )
        }
        val mimicEl = el.firstChild("mimic")
        var mimic: MimicSpec? = null
        if (mimicEl != null) {
            val target = mimicEl.attr("joint")
            if (target == null) {
                warnings.add("joint $name 的 <mimic> 缺少 joint 属性")
            } else {
                mimic = MimicSpec(
                    jointName = target,
                    multiplier = mimicEl.attr("multiplier")?.toDoubleOrNull() ?: 1.0,
                    offsetRadiansOrMeters = mimicEl.attr("offset")?.toDoubleOrNull() ?: 0.0,
                )
            }
        }
        return JointSpec(name, type, parent, child, origin, axis, limit, mimic, el, idx)
    }

    private fun parseOrigin(el: XmlElement, where: String, warnings: MutableList<String>): OriginSpec {
        val xyz = parseVec(el.attr("xyz"))
        val rpy = parseVec(el.attr("rpy"))
        if (el.attr("xyz") != null && xyz == null) warnings.add("$where 的 xyz 无法解析")
        if (el.attr("rpy") != null && rpy == null) warnings.add("$where 的 rpy 无法解析")
        return OriginSpec(
            xyz = xyz ?: Vec3Spec(0.0, 0.0, 0.0),
            rpyRadians = rpy ?: Vec3Spec(0.0, 0.0, 0.0),
            element = el,
        )
    }

    private fun parseVec(raw: String?): Vec3Spec? {
        if (raw.isNullOrBlank()) return null
        val parts = raw.trim().split(Regex("\\s+"))
        if (parts.size != 3) return null
        val nums = parts.map { it.toDoubleOrNull() ?: return null }
        return Vec3Spec(nums[0], nums[1], nums[2])
    }
}
