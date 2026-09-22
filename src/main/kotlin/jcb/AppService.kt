package jcb

import jcb.calib.CalibStatus
import jcb.calib.CalibrationService
import jcb.calib.CalibrationTransform
import jcb.calib.EffectiveCalibration
import jcb.db.Database
import jcb.db.RobotRecord
import jcb.db.SnapshotRecord
import jcb.kin.CycleAnalyzer
import jcb.kin.FrameEdge
import jcb.kin.FrameGraph
import jcb.kin.FrameQueryEngine
import jcb.kin.JointUnit
import jcb.kin.JointValue
import jcb.urdf.UrdfModel
import jcb.urdf.UrdfParser
import jcb.xml.XmlElement
import jcb.xml.XmlRaw
import jcb.xml.XmlText
import kotlinx.serialization.builtins.MapSerializer
import kotlinx.serialization.builtins.serializer
import java.time.Instant

class VersionConflictException(message: String) : RuntimeException(message)

data class ImportResult(
    val robot: RobotRecord,
    val model: UrdfModel,
    val graph: FrameGraph,
    val issues: List<FrameGraphIssueDto>,
)

data class FrameGraphIssueDto(val kind: String, val severity: String, val message: String, val ref: String?)

@kotlinx.serialization.Serializable
data class JointValueInput(val value: Double, val unit: String = "rad")

data class EvaluationResult(
    val selectorLabel: String,
    val effective: List<EffectiveCalibration>,
    val query: jcb.kin.FrameQueryResult,
    val cycles: jcb.kin.CycleReport,
    val graphIssues: List<FrameGraphIssueDto>,
)

class AppService(val db: Database) {
    private data class Loaded(val record: RobotRecord, val doc: jcb.xml.XmlDocument, val model: UrdfModel)

    private fun load(serial: String): Loaded {
        val record = db.getRobotBySerial(serial)
            ?: throw IllegalArgumentException("机器人序列号 '$serial' 不存在")
        val (doc, model) = UrdfParser.parse(record.urdfXml)
        return Loaded(record, doc, model)
    }

    fun importRobot(serial: String, name: String?, xml: String): ImportResult {
        if (db.getRobotBySerial(serial) != null)
            throw IllegalArgumentException("机器人序列号 '$serial' 已存在")
        val (doc, model) = UrdfParser.parse(xml)
        val built = FrameGraph.build(model)
        val record = db.importRobot(serial, name ?: model.robotName, xml)
        return ImportResult(
            record, model, built.graph,
            built.graph.issues.map { FrameGraphIssueDto(it.kind, it.severity, it.message, it.ref) }
        )
    }

    fun listRobots() = db.listRobots()

    fun createSnapshot(
        serial: String, label: String, takenAt: String?, values: Map<String, JointValueInput>
    ): Long {
        val robot = db.getRobotBySerial(serial)
            ?: throw IllegalArgumentException("机器人 '$serial' 不存在")
        val json = json().encodeToString(
            MapSerializer(String.serializer(), JointValueInput.serializer()), values
        )
        return db.insertSnapshot(
            robot.id, label, takenAt?.let(Instant::parse), json, frozen = false
        )
    }

    fun freezeSnapshot(snapshotId: Long) {
        val snap = db.getSnapshot(snapshotId)
            ?: throw IllegalArgumentException("快照 #$snapshotId 不存在")
        if (snap.frozen) throw IllegalArgumentException("快照 #$snapshotId 已冻结，不能重复冻结")
        db.setSnapshotFrozen(snapshotId, true)
    }

    fun listSnapshots(serial: String): List<SnapshotRecord> {
        val robot = db.getRobotBySerial(serial)
            ?: throw IllegalArgumentException("机器人 '$serial' 不存在")
        return db.listSnapshots(robot.id)
    }

    /**
     * 创建或更新草案。baseVersion 为乐观锁：
     * 必须等于该标定 id 当前最新已批准版本，否则 409。
     */
    fun saveDraft(
        id: String,
        serial: String,
        baseVersion: Int,
        parentFrame: String,
        childFrame: String,
        xyz: List<Double>,
        rpyRad: List<Double>,
        note: String?,
        validFrom: String?,
        validTo: String?,
    ): CalibrationTransform {
        require(xyz.size == 3 && rpyRad.size == 3) { "xyz/rpy 必须各为 3 个分量" }
        val latest = db.listCalibrations().filter { it.id == id }.maxByOrNull { it.version }
        val expectedBase = latest?.takeIf { it.status == CalibStatus.APPROVED }?.version ?: 0
        if (baseVersion != expectedBase) {
            throw VersionConflictException(
                "版本冲突：草案基于 v$baseVersion，但 '$id' 当前最新已批准版本为 v$expectedBase；请刷新后重新编辑"
            )
        }
        val now = Instant.now()
        // 已有 DRAFT（基于同一版本）则覆盖，否则新增草案行，版本号 = 最新+1
        val existingDraft = latest?.takeIf { it.status == CalibStatus.DRAFT && it.baseVersion == baseVersion }
        val draft = CalibrationTransform(
            id = id,
            version = existingDraft?.version ?: ((latest?.version ?: 0) + 1),
            robotSerial = serial,
            parentFrame = parentFrame,
            childFrame = childFrame,
            xyz = xyz.toDoubleArray(),
            rpyRad = rpyRad.toDoubleArray(),
            note = note,
            validFrom = validFrom?.let(Instant::parse),
            validTo = validTo?.let(Instant::parse),
            createdAt = now,
            status = CalibStatus.DRAFT,
            baseVersion = baseVersion,
        )
        db.saveCalibration(draft)
        return draft
    }

    /** 批准草案：旧 APPROVED 置 SUPERSEDED，草案转 APPROVED 并升版为最新+1。 */
    fun approve(id: String, expectedDraftVersion: Int): CalibrationTransform {
        val all = db.listCalibrations().filter { it.id == id }
        val draft = all.firstOrNull { it.version == expectedDraftVersion && it.status == CalibStatus.DRAFT }
            ?: throw IllegalArgumentException("'$id' 不存在版本 v$expectedDraftVersion 的草案")
        val currentApproved = all.filter { it.status == CalibStatus.APPROVED }
        if (draft.baseVersion != (currentApproved.maxOfOrNull { it.version } ?: 0)) {
            throw VersionConflictException("批准失败：草案基线 v${draft.baseVersion} 已过期")
        }
        currentApproved.forEach { db.saveCalibration(it.copy(status = CalibStatus.SUPERSEDED)) }
        val approvedVersion = (all.maxOf { it.version } + 1).let { v -> if (v <= expectedDraftVersion) expectedDraftVersion + 1 else v }
        val approved = draft.copy(version = approvedVersion, status = CalibStatus.APPROVED)
        db.saveCalibration(approved)
        return approved
    }

    fun reject(id: String, version: Int) {
        val all = db.listCalibrations()
        val draft = all.firstOrNull { it.id == id && it.version == version }
            ?: throw IllegalArgumentException("标定版本不存在")
        db.saveCalibration(draft.copy(status = CalibStatus.REJECTED))
    }

    data class Selection(val picked: List<CalibrationTransform>, val effective: List<EffectiveCalibration>, val label: String)

    fun selectCalibrations(
        serial: String,
        ids: List<String>?,
        at: Instant?,
    ): Selection {
        val all = db.listCalibrations(serial)
        if (!ids.isNullOrEmpty()) {
            // 显式指定“标定 id@version”，未指定版本时取该 id 最新版本（允许比较未批准版本）
            val picked = ids.map { token ->
                val parts = token.split("@")
                val cid = parts[0]
                val versions = all.filter { it.id == cid }
                when {
                    parts.size == 2 -> versions.firstOrNull { it.version == parts[1].toInt() }
                        ?: throw IllegalArgumentException("标定 $token 不存在")
                    else -> versions.maxByOrNull { it.version }
                        ?: throw IllegalArgumentException("标定 $cid 不存在")
                }
            }
            val effective = picked.map {
                CalibrationService.evaluate(listOf(it), serial, at).single()
            }
            return Selection(picked, effective, "指定版本: " + ids.joinToString())
        }
        val approved = all.filter { it.status == CalibStatus.APPROVED }
        val effective = CalibrationService.evaluate(approved, serial, at)
        return Selection(approved, effective, "最新已批准标定")
    }

    data class Scenario(
        val robotSerial: String,
        val snapshotId: Long?,
        val from: String,
        val to: String,
        val calibrationIds: List<String>?,
        val inlineValues: Map<String, JointValueInput>?,
        val toleranceM: Double = 1e-6,
        val toleranceRad: Double = 1e-6,
    )

    fun evaluate(scenario: Scenario): EvaluationResult {
        val loaded = load(scenario.robotSerial)
        val (values, at) = resolveValues(scenario, loaded.record.id)
        val selection = selectCalibrations(scenario.robotSerial, scenario.calibrationIds, at)
        val effective = selection.effective
        val label = selection.label
        val calibEdges = CalibrationService.toEdges(effective)
        val built = FrameGraph.build(loaded.model, calibEdges)
        val mimic = loaded.model.joints.associateBy { it.name }
        val query = FrameQueryEngine(built.graph).query(
            scenario.from, scenario.to, values, mimic
        )
        val cycles = CycleAnalyzer(scenario.toleranceM, scenario.toleranceRad)
            .analyze(built.graph, values, mimic)
        return EvaluationResult(
            selection.label,
            effective,
            query,
            cycles,
            built.graph.issues.map { FrameGraphIssueDto(it.kind, it.severity, it.message, it.ref) }
        )
    }

    data class CompareResult(
        val a: EvaluationResult,
        val b: EvaluationResult,
        val diffs: List<jcb.calib.CalibDiffEntry>,
    )

    fun compare(
        base: Scenario,
        other: Scenario,
    ): CompareResult {
        val a = evaluate(base)
        val b = evaluate(other)
        val diffs = CalibrationService.diff(
            a.effective.map { it.transform },
            b.effective.map { it.transform },
        )
        return CompareResult(a, b, diffs)
    }

    private fun resolveValues(
        scenario: Scenario, robotId: Long
    ): Pair<Map<String, JointValue>, Instant?> {
        val merged = sortedMapOf<String, JointValueInput>()
        var at: Instant? = null
        if (scenario.snapshotId != null) {
            val snap = db.getSnapshot(scenario.snapshotId)
                ?: throw IllegalArgumentException("快照 #${scenario.snapshotId} 不存在")
            if (snap.robotId != robotId)
                throw IllegalArgumentException("快照不属于机器人 ${scenario.robotSerial}")
            val parsed: Map<String, JointValueInput> = json().decodeFromString(
                MapSerializer(String.serializer(), JointValueInput.serializer()), snap.jointValuesJson
            )
            merged.putAll(parsed)
            at = snap.takenAt
        }
        scenario.inlineValues?.let { merged.putAll(it) }
        val resolved = merged.map { (name, v) ->
            val unit = when (v.unit.lowercase()) {
                "deg", "degree", "degrees" -> JointUnit.DEGREE
                "m", "meter", "meters" -> JointUnit.METER
                else -> JointUnit.RADIAN
            }
            name to JointValue(name, v.value, unit)
        }.toMap()
        return resolved to at
    }

    /**
     * 导出 XML：只把“已批准”补丁应用到原 URDF 的工作副本，
     * 保留全部未知元素/属性/原始顺序，并以注释标明派生来源，绝不冒充原 URDF。
     */
    fun exportDerivedUrdf(serial: String, calibIds: List<String>?): String {
        val loaded = load(serial)
        val approved = db.listCalibrations(serial).filter { it.status == CalibStatus.APPROVED }
        val chosen = if (calibIds.isNullOrEmpty()) approved
        else calibIds.map { token ->
            val cid = token.substringBefore("@")
            val ver = token.substringAfter("@", "").toIntOrNull()
            approved.filter { it.id == cid && (ver == null || it.version == ver) }
                .maxByOrNull { it.version }
                ?: throw IllegalArgumentException("没有已批准的标定 $token 可导出")
        }
        val provenance = buildList {
            add("关节坐标册：派生 URDF —— 原始 URDF 未被修改")
            add("机器人序列号: $serial；生成时间: ${Instant.now()}")
            add("已应用获批补丁: " + chosen.joinToString { "${it.id}@v${it.version}" })
        }
        val root = loaded.doc.root
        provenance.asReversed().forEach { line ->
            root.children.add(0, XmlRaw("<!--$line-->"))
            root.children.add(1, XmlText("\n  "))
        }
        for (c in chosen) applyPatch(root, c)
        return loaded.doc.toXmlString()
    }

    private fun applyPatch(root: XmlElement, c: CalibrationTransform) {
        val existing = root.elements("joint").firstOrNull { j ->
            j.element("parent")?.attr("link") == c.parentFrame &&
                j.element("child")?.attr("link") == c.childFrame
        }
        if (existing != null) {
            val origin = existing.getOrCreateChild("origin")
            origin.setAttr("xyz", c.xyz.joinToString(" ") { "%.9f".format(it).trimEnd('0').trimEnd('.') })
            origin.setAttr("rpy", c.rpyRad.joinToString(" ") { "%.9f".format(it).trimEnd('0').trimEnd('.') })
            origin.attributes.put("jcb:source", "calibration:${c.id}@v${c.version}")
            return
        }
        // 同 pair 无 joint：追加固定关节（仍保留为显式补充而非伪装原内容）
        val joint = XmlElement(
            "joint", linkedMapOf(
                "name" to "jcb_calib_${c.id}",
                "type" to "fixed",
                "jcb:source" to "calibration:${c.id}@v${c.version}",
            )
        )
        joint.children += XmlText("\n    ")
        joint.children += XmlElement("origin", linkedMapOf(
            "xyz" to c.xyz.joinToString(" "),
            "rpy" to c.rpyRad.joinToString(" "),
        ))
        joint.children += XmlText("\n    ")
        joint.children += XmlElement("parent", linkedMapOf("link" to c.parentFrame))
        joint.children += XmlText("\n    ")
        joint.children += XmlElement("child", linkedMapOf("link" to c.childFrame))
        joint.children += XmlText("\n  ")
        root.children.add(XmlText("\n  "))
        root.children.add(joint)
        root.children.add(XmlText("\n"))
    }

    private fun json() = kotlinx.serialization.json.Json { ignoreUnknownKeys = true }
}
