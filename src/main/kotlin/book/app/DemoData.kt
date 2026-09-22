package book.app

import book.calib.CalibrationEdge
import book.calib.CalibrationVersion
import book.db.Database
import java.time.Instant
import java.time.temporal.ChronoUnit
import java.util.UUID

/**
 * 演示机器人 ARM-DEMO-001：
 *  - base→L1→L2→L3→tool 为主链（含 revolute / continuous / prismatic）
 *  - j2 被 j2_mimic 跟随（mimic 链）
 *  - L1→L2 之间有两条额外路径：一条闭合一致，一条人为制造 0.02 m 矛盾环
 *  - ghost_link 被关节引用但未定义（断链）
 *  - 顶层保留一个未知元素 <manufacturer:note>（未知内容往返示例）
 */
object DemoData {

    const val SERIAL = "ARM-DEMO-001"

    val urdf = """
<?xml version="1.0"?>
<!-- 演示 URDF：包含 mimic、环路与断链；本注释在导出时必须保留 -->
<robot name="demo_arm">
  <manufacturer:note vendor="acme">顶层未知元素，往返保留示例</manufacturer:note>

  <link name="base">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="10"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
    <visual><geometry><box size="0.2 0.2 0.1"/></geometry></visual>
  </link>

  <link name="L1">
    <inertial><mass value="2"/><inertia ixx="0.01" iyy="0.01" izz="0.01"/></inertial>
    <visual><geometry><cylinder radius="0.05" length="0.2"/></geometry></visual>
  </link>
  <link name="L2">
    <visual><geometry><sphere radius="0.06"/></geometry></visual>
  </link>
  <link name="L3">
    <visual><geometry><mesh filename="package://demo/meshes/l3.stl" scale="1 1 1"/></geometry></visual>
  </link>
  <link name="tool"/>
  <link name="strutA"/>
  <link name="strutB"/>

  <joint name="j1" type="revolute">
    <parent link="base"/><child link="L1"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14159" upper="3.14159" effort="50" velocity="1.5"/>
  </joint>

  <joint name="j2" type="revolute">
    <parent link="L1"/><child link="L2"/>
    <origin xyz="0 0 0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="30" velocity="1.0"/>
  </joint>

  <joint name="j2_mimic" type="revolute">
    <parent link="L1"/><child link="L2"/>
    <origin xyz="0 0 0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <mimic joint="j2" multiplier="1.0" offset="0.0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
  </joint>

  <joint name="j3" type="prismatic">
    <parent link="L2"/><child link="L3"/>
    <origin xyz="0.05 0 0" rpy="0 0 1.57079632679"/>
    <axis xyz="1 0 0"/>
    <limit lower="0" upper="0.3" effort="20" velocity="0.2"/>
  </joint>

  <joint name="jtool" type="fixed">
    <parent link="L3"/><child link="tool"/>
    <origin xyz="0.1 0 0" rpy="0 0 0"/>
  </joint>

  <!-- 一致闭合环：L1 -> strutA -> strutB -> L2，与 j2 路径几何一致 -->
  <joint name="s1" type="fixed">
    <parent link="L1"/><child link="strutA"/>
    <origin xyz="0.2 0 0.1" rpy="0 0 0"/>
  </joint>
  <joint name="s2" type="fixed">
    <parent link="strutA"/><child link="strutB"/>
    <origin xyz="-0.2 0 0.3" rpy="0 0 0"/>
  </joint>
  <joint name="s3" type="fixed">
    <parent link="strutB"/><child link="L2"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>

  <!-- 断链：parent 未定义 -->
  <joint name="j_broken" type="fixed">
    <parent link="ghost_link"/><child link="L3"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
</robot>
""".trimIndent()

    fun seed(db: Database) {
        val robot = db.upsertRobot(SERIAL, "演示六坐标臂（含环路/mimic/断链）", urdf)
        val now = Instant.now().truncatedTo(ChronoUnit.SECONDS)

        val v1 = CalibrationVersion(
            id = UUID.randomUUID().toString(), robotSerial = SERIAL, version = 1,
            note = "出厂标定", createdAt = now.minus(30, ChronoUnit.DAYS).toString(),
            validFrom = now.minus(20, ChronoUnit.DAYS).toString(),
            validUntil = now.plus(10, ChronoUnit.DAYS).toString(),
            draft = false, approved = true,
            edges = listOf(
                CalibrationEdge(
                    id = "c-j2-origin", kind = "override", jointName = "j2",
                    parentFrame = "L1", childFrame = "L2",
                    xyz = listOf(0.0, 0.0, 0.4), rpyRad = listOf(0.0, 0.0, 0.0),
                    note = "出厂原点",
                ),
                CalibrationEdge(
                    id = "c-brace", kind = "new", jointName = null,
                    parentFrame = "base", childFrame = "strutA",
                    xyz = listOf(0.2, 0.0, 0.4), rpyRad = listOf(0.0, 0.0, 0.0),
                    note = "现场加装支撑（与 URDF 路径并列，用于多路径一致性展示）",
                ),
            ),
        )
        db.insertCalibrationVersion(v1, robot.id)

        val v2 = CalibrationVersion(
            id = UUID.randomUUID().toString(), robotSerial = SERIAL, version = 2,
            note = "现场修正 j2 偏移 2cm（制造矛盾环以便演示残差）",
            createdAt = now.minus(1, ChronoUnit.DAYS).toString(),
            validFrom = now.minus(1, ChronoUnit.DAYS).toString(),
            validUntil = now.plus(30, ChronoUnit.DAYS).toString(),
            draft = false, approved = true,
            edges = listOf(
                CalibrationEdge(
                    id = "c-j2-origin", kind = "override", jointName = "j2",
                    parentFrame = "L1", childFrame = "L2",
                    xyz = listOf(0.02, 0.0, 0.4), rpyRad = listOf(0.0, 0.0, 0.0),
                    note = "j2 原点 x+0.02",
                ),
                CalibrationEdge(
                    id = "c-brace", kind = "new", jointName = null,
                    parentFrame = "base", childFrame = "strutA",
                    xyz = listOf(0.2, 0.0, 0.4), rpyRad = listOf(0.0, 0.0, 0.0),
                    note = "现场加装支撑",
                ),
            ),
        )
        db.insertCalibrationVersion(v2, robot.id)

        val v3Future = CalibrationVersion(
            id = UUID.randomUUID().toString(), robotSerial = SERIAL, version = 3,
            note = "计划中的下周标定（FUTURE 状态演示）",
            createdAt = now.toString(),
            validFrom = now.plus(7, ChronoUnit.DAYS).toString(),
            validUntil = now.plus(40, ChronoUnit.DAYS).toString(),
            draft = false, approved = true,
            edges = v2.edges,
        )
        db.insertCalibrationVersion(v3Future, robot.id)
    }
}
