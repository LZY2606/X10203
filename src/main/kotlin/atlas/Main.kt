package atlas

import atlas.kinematics.CalibrationSet
import atlas.kinematics.CalibrationTransform
import atlas.server.JointAtlasServer
import atlas.store.Snapshot
import atlas.store.Store

val DEMO_URDF = """<?xml version="1.0"?>
<robot name="demo_cell" xmlns:vendor="https://example.com/vendor">
  <vendor:siteMeta cell="A7" shift="night"/>
  <link name="base_link">
    <inertial><mass value="2.0"/><origin xyz="0 0 0.05" rpy="0 0 0"/></inertial>
    <visual><geometry><box size="0.2 0.2 0.1"/></geometry></visual>
  </link>
  <link name="link1">
    <visual><geometry><cylinder radius="0.05" length="0.4"/></geometry></visual>
  </link>
  <link name="link2"/>
  <link name="aux"/>
  <link name="tip"/>
  <joint name="j1" type="revolute">
    <parent link="base_link"/><child link="link1"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1.0"/>
  </joint>
  <joint name="j2" type="revolute">
    <parent link="link1"/><child link="link2"/>
    <origin xyz="0 0 0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
  </joint>
  <joint name="j3" type="fixed">
    <parent link="link2"/><child link="tip"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
  </joint>
  <joint name="j4" type="fixed">
    <parent link="link1"/><child link="aux"/>
    <origin xyz="0.1 0 0.2" rpy="0 0 0"/>
  </joint>
  <joint name="j5" type="fixed">
    <parent link="aux"/><child link="link2"/>
    <origin xyz="-0.1 0 0.2" rpy="0 0 0"/>
  </joint>
  <joint name="j6" type="revolute">
    <parent link="tip"/><child link="gripper"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <mimic joint="j2" multiplier="2.0" offset="0.1"/>
  </joint>
  <link name="gripper"/>
</robot>
"""

fun main(args: Array<String>) {
    var host = "127.0.0.1"
    var port = 5263
    var db = "jointatlas.db"
    var i = 0
    while (i < args.size) {
        when (args[i]) {
            "--host" -> host = args[++i]
            "--port" -> port = args[++i].toInt()
            "--db" -> db = args[++i]
        }
        i++
    }
    val store = Store(db)
    if (store.listUrdf().isEmpty()) seed(store)
    val server = JointAtlasServer.create(store, host, port)
    println("关节坐标册 listening on http://$host:$port")
    server.start(wait = true)
}

private fun seed(store: Store) {
    store.insertUrdf("demo_cell", DEMO_URDF)
    val calId = store.insertCalibration(
        CalibrationSet(
            id = 0, name = "现场标定A", robotSerial = "DEMO-1",
            validFrom = 1_000L, validTo = 9_999_999_999_999L,
            transforms = listOf(
                CalibrationTransform("base_link", "tool_cal", listOf(0.0, 0.0, 0.1), listOf(0.0, 0.0, 0.0)),
                CalibrationTransform("link2", "tool_cal", listOf(0.0, 0.0, 0.5), listOf(0.0, 0.0, 0.0))
            )
        )
    )
    store.insertSnapshot(
        Snapshot(
            id = 0, name = "现场快照-1", robotSerial = "DEMO-1",
            jointValues = mapOf("j1" to 0.5, "j2" to -0.3),
            capturedAt = 5_000L, calibrationId = calId
        )
    )
}
