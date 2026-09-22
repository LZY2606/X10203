package atlas

enum class ApproveResult { OK, CONFLICT, NOT_FOUND, NOT_PENDING, BAD_PATCH }

data class ImportResult(val version: Int, val diagnostics: List<String>)

class Atlas(val store: Store) {

    fun importUrdf(name: String, xml: String): ImportResult {
        val doc = Xml.parse(xml)
        val model = parseUrdf(doc)
        val version = store.addUrdf(name.ifBlank { model.robotName }, xml)
        val diags = model.duplicateFrames.map { "重复 frame: $it" }
        return ImportResult(version, diags)
    }

    fun currentModel(): Pair<UrdfVersion, UrdfModel>? {
        val v = store.getUrdf(null) ?: return null
        return v to parseUrdf(Xml.parse(v.xml))
    }

    fun engine(serial: String, time: Long): Engine? {
        val (_, model) = currentModel() ?: return null
        return Engine(model, store.calibs(), serial, time)
    }

    fun addCalib(
        id: String, from: String, to: String,
        xyz: DoubleArray, rpy: DoubleArray, unit: String,
        serial: String, validFrom: Long, validTo: Long
    ): CalibTransform {
        val c = CalibTransform(id, store.nextCalibVersion(id), from, to, xyz, rpy, unit, serial, validFrom, validTo)
        store.addCalib(c)
        return c
    }

    /** 草案审批：版本号并发控制，baseVersion 必须等于当前版本，否则冲突。 */
    fun approveDraft(id: Int): ApproveResult {
        val draft = store.getDraft(id) ?: return ApproveResult.NOT_FOUND
        if (draft.status != "PENDING") return ApproveResult.NOT_PENDING
        val current = store.getUrdf(null) ?: return ApproveResult.NOT_FOUND
        if (draft.baseVersion != current.id) return ApproveResult.CONFLICT
        val patched = try {
            applyPatch(current.xml, draft.patch)
        } catch (e: Exception) {
            return ApproveResult.BAD_PATCH
        }
        store.addUrdf("${current.name}+draft${draft.id}", patched)
        store.setDraftStatus(id, "APPROVED")
        return ApproveResult.OK
    }

    fun rejectDraft(id: Int): Boolean {
        val draft = store.getDraft(id) ?: return false
        if (draft.status != "PENDING") return false
        store.setDraftStatus(id, "REJECTED")
        return true
    }

    /**
     * 在保留未知内容的 DOM 上应用补丁，然后整体序列化导出。
     * 支持操作：<set-joint-origin joint=".." xyz=".." rpy=".."/> 与
     * <add-joint> ...原始 joint XML... </add-joint>。
     */
    fun applyPatch(xml: String, patchXml: String): String {
        val doc = Xml.parse(xml)
        val robot = Xml.document(doc)
        val patch = Xml.document(Xml.parse(patchXml))
        require(patch.name == "patch") { "补丁根元素必须是 <patch>" }
        for (op in patch.children.filterIsInstance<XElement>()) {
            when (op.name) {
                "set-joint-origin" -> {
                    val jointName = op.attr("joint") ?: throw IllegalArgumentException("缺少 joint 属性")
                    val joint = robot.children("joint").firstOrNull { it.attr("name") == jointName }
                        ?: throw IllegalArgumentException("未找到关节 $jointName")
                    val origin = joint.child("origin") ?: XElement("origin").also { joint.children.add(it) }
                    op.attr("xyz")?.let { origin.attrs["xyz"] = it }
                    op.attr("rpy")?.let { origin.attrs["rpy"] = it }
                }
                "add-joint" -> {
                    val j = op.children.filterIsInstance<XElement>().firstOrNull()
                        ?: throw IllegalArgumentException("add-joint 缺少关节定义")
                    robot.children.add(j)
                }
                else -> throw IllegalArgumentException("未知补丁操作: ${op.name}")
            }
        }
        return Xml.serialize(doc)
    }

    /** 简单 LCS 行 diff，用于版本差异展示。 */
    fun diff(v1: Int, v2: Int): List<String>? {
        val a = store.getUrdf(v1) ?: return null
        val b = store.getUrdf(v2) ?: return null
        return diffLines(a.xml.lines(), b.xml.lines())
    }

    companion object {
        fun diffLines(a: List<String>, b: List<String>): List<String> {
            val n = a.size
            val m = b.size
            val dp = Array(n + 1) { IntArray(m + 1) }
            for (i in n - 1 downTo 0) {
                for (j in m - 1 downTo 0) {
                    dp[i][j] = if (a[i] == b[j]) dp[i + 1][j + 1] + 1 else maxOf(dp[i + 1][j], dp[i][j + 1])
                }
            }
            val out = mutableListOf<String>()
            var i = 0
            var j = 0
            while (i < n && j < m) {
                when {
                    a[i] == b[j] -> { out.add("  ${a[i]}"); i++; j++ }
                    dp[i + 1][j] >= dp[i][j + 1] -> { out.add("- ${a[i]}"); i++ }
                    else -> { out.add("+ ${b[j]}"); j++ }
                }
            }
            while (i < n) { out.add("- ${a[i]}"); i++ }
            while (j < m) { out.add("+ ${b[j]}"); j++ }
            return out
        }
    }
}
