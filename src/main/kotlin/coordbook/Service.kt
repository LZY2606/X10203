package coordbook

/**
 * 应用服务层：组合 URDF 解析、标定生效判定、快照与坐标查询。
 */
class RobotService(private val db: Database) {

    fun parseRobot(robot: Robot): Pair<UrdfDocument, List<DocumentIssue>> {
        val doc = UrdfParser.parse(robot.urdfXml)
        return doc to doc.issues
    }

    fun graphFor(
        robot: Robot,
        snapshot: Snapshot?,
        atIso: String,
        calibVersionId: Long?,
    ): EffectiveGraph {
        val doc = UrdfParser.parse(robot.urdfXml)
        val versions = db.listCalibVersions(robot.id)
        val patches = db.listPatches()
        val serial = snapshot?.serial ?: robot.serial
        return if (calibVersionId != null) {
            val pinned = db.getCalibVersion(calibVersionId)
                ?: throw NoSuchElementException("标定版本 #$calibVersionId 不存在")
            val effective = CalibrationService.effectiveAt(pinned, serial, atIso)
            val (graph, issues) = GraphBuilder.build(doc)
            val approved = patches.filter {
                it.versionId == pinned.id && it.status == PatchStatus.APPROVED
            }
            val calibIssues = CalibrationService.applyApproved(
                graph, approved, pinned.id, pinned.revision,
            )
            EffectiveGraph(graph, pinned, effective, approved.map { it.id }, issues + calibIssues)
        } else {
            SnapshotService.buildEffectiveGraph(doc, versions, patches, serial, atIso)
        }
    }

    fun query(
        robot: Robot,
        snapshot: Snapshot?,
        from: String,
        to: String,
        atIso: String,
        valuesOverride: Map<String, Double>?,
        calibVersionId: Long?,
    ): FrameQueryResult {
        val doc = UrdfParser.parse(robot.urdfXml)
        val effective = graphFor(robot, snapshot, atIso, calibVersionId)
        val values = valuesOverride ?: snapshot?.jointValues ?: emptyMap()
        return Kinematics.query(effective.graph, doc, from, to, values)
    }

    fun cycleReport(
        robot: Robot,
        atIso: String,
        calibVersionId: Long?,
    ): CycleReport {
        val effective = graphFor(robot, null, atIso, calibVersionId)
        return Cycles.analyze(effective.graph)
    }

    fun seedIfEmpty() {
        if (db.listRobots().isNotEmpty()) return
        val robot = db.insertRobot("演示机械臂", SeedData.SERIAL, SeedData.SEED_URDF)
        db.insertSnapshot(
            robot.id, SeedData.SERIAL, "2026-09-21T08:00:00Z",
            SeedData.SEED_VALUES, "产线标定前冻结快照",
        )
        val v1 = db.insertCalibVersion(
            robot.id, SeedData.SERIAL,
            "2026-09-01T00:00:00Z", "2026-12-31T23:59:59Z",
            "初版现场标定",
        )
        db.insertPatch(
            v1.id, PatchKind.JOINT_ORIGIN_RPY, "j1", null, null,
            null, doubleArrayOf(0.0, 0.0, 0.002), "j1 零位偏置修正",
        )
        db.insertPatch(
            v1.id, PatchKind.EXTRA_TRANSFORM, null, "base", "brace_top",
            doubleArrayOf(0.1, 0.0, 0.35), doubleArrayOf(0.0, 0.0, 0.0),
            "复测得到的同一变换（与 URDF 平行声明）",
        )
        // 审批上述两个补丁
        for (patch in db.listPatches(v1.id)) {
            db.setPatchStatus(patch.id, PatchStatus.APPROVED, v1.revision)
        }
        val v2 = db.insertCalibVersion(
            robot.id, SeedData.SERIAL,
            "2027-01-01T00:00:00Z", "2027-06-30T23:59:59Z",
            "下一阶段草案（当前时间窗之外）",
        )
        db.insertPatch(
            v2.id, PatchKind.JOINT_ORIGIN_XYZ, "j2",
            null, null, doubleArrayOf(0.0, 0.0, 0.249), null,
            "待评审的基座高度修正",
        )
        db.insertRobot("不一致环路样机", SeedData.SECOND_SERIAL, SeedData.SEED_URDF_BAD)
    }
}
