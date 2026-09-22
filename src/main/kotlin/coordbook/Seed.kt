package coordbook

object Seed {
    const val DEMO_SERIAL = "SN-001"

    val DEMO_URDF = """<?xml version="1.0"?>
<robot name="demo_arm" vendor:tag="keep-me" xmlns:vendor="http://example.com/vendor">
  <!-- unknown elements below must round-trip untouched -->
  <gazebo reference="base_link"><material>Gazebo/Grey</material></gazebo>
  <link name="base_link">
    <inertial><mass value="2.0"/><origin xyz="0 0 0.05" rpy="0 0 0"/><inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial>
    <visual><geometry><box size="0.1 0.1 0.1"/></geometry></visual>
  </link>
  <link name="link1">
    <visual><geometry><cylinder radius="0.03" length="0.3"/></geometry></visual>
    <custom_field fiber="carbon"/>
  </link>
  <link name="link2">
    <visual><geometry><cylinder radius="0.03" length="0.3"/></geometry></visual>
  </link>
  <link name="link3">
    <visual><geometry><sphere radius="0.05"/></geometry></visual>
  </link>
  <link name="tool0">
    <visual><geometry><mesh filename="package://demo/tool.stl" scale="1 1 1"/></geometry></visual>
  </link>
  <joint name="j1" type="revolute">
    <parent link="base_link"/><child link="link1"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1.5"/>
  </joint>
  <joint name="j2" type="revolute">
    <parent link="link1"/><child link="link2"/>
    <origin xyz="0.3 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1.5"/>
  </joint>
  <joint name="j3" type="revolute">
    <parent link="link2"/><child link="link3"/>
    <origin xyz="0.3 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="5" velocity="1.5"/>
    <mimic joint="j2" multiplier="1" offset="0"/>
  </joint>
  <joint name="j4" type="fixed">
    <parent link="link3"/><child link="tool0"/>
    <origin xyz="0.2 0 0.05" rpy="0 0 0"/>
  </joint>
  <transmission name="tran1"><type>transmission_interface/SimpleTransmission</type></transmission>
</robot>
"""

    fun seedIfEmpty(db: Db) {
        if (db.latestUrdf(DEMO_SERIAL) != null) return
        db.insertUrdf("demo_arm", DEMO_SERIAL, 1, DEMO_URDF)
        // Consistent loop closure at zero posture: tool0 -> base_link = inverse of chain (0.3+0.3+0.2, 0.1+0.05).
        db.insertCalibration(
            Calibration(0, DEMO_SERIAL, "loop_closure", "tool0", "base_link",
                doubleArrayOf(-0.8, 0.0, -0.15), doubleArrayOf(0.0, 0.0, 0.0), "rad",
                null, null, 1)
        )
        // Duplicate declaration of j1 by calibration, slightly off -> inconsistent 2-cycle.
        db.insertCalibration(
            Calibration(0, DEMO_SERIAL, "shoulder_resurvey", "base_link", "link1",
                doubleArrayOf(0.0, 0.0, 0.1005), doubleArrayOf(0.0, 0.0, 0.0), "rad",
                null, null, 1)
        )
        // Degree-unit camera mount, time-bounded.
        db.insertCalibration(
            Calibration(0, DEMO_SERIAL, "camera_mount", "base_link", "camera_frame",
                doubleArrayOf(0.05, 0.0, 0.2), doubleArrayOf(0.0, 0.0, 90.0), "deg",
                1700000000L, 1900000000L, 1)
        )
        db.insertSnapshot("home", DEMO_SERIAL, mapOf("j1" to 0.0, "j2" to 0.0), 1750000000L)
    }
}
