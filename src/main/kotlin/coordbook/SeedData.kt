package coordbook

object SeedData {

    const val SERIAL = "SN-DEMO-007"

    val SEED_URDF: String = """<?xml version="1.0"?>
<robot name="demo_arm">
  <!-- 演示机器人：含 mimic 链、一致固定环路、平行路径与厂商未知元素 -->
  <link name="base">
    <inertial>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <mass value="5.2"/>
      <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.03"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><cylinder radius="0.12" length="0.1"/></geometry>
    </visual>
  </link>
  <link name="shoulder">
    <visual><geometry><sphere radius="0.08"/></geometry></visual>
  </link>
  <link name="elbow">
    <visual><geometry><box size="0.1 0.1 0.25"/></geometry></visual>
  </link>
  <link name="wrist">
    <visual><geometry><mesh filename="package://demo/meshes/wrist.stl"/></geometry></visual>
  </link>
  <link name="brace_top"/>
  <link name="brace_bottom"/>
  <joint name="j1" type="revolute">
    <parent link="base"/>
    <child link="shoulder"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14159265358979" upper="3.14159265358979" effort="50" velocity="1.2"/>
  </joint>
  <joint name="j2" type="revolute">
    <parent link="shoulder"/>
    <child link="elbow"/>
    <origin xyz="0 0 0.25" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.5" upper="2.5" effort="30" velocity="1.0"/>
  </joint>
  <joint name="j3" type="revolute">
    <parent link="elbow"/>
    <child link="wrist"/>
    <origin xyz="0 0 0.25" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.0" upper="2.0" effort="20" velocity="1.5"/>
    <mimic joint="j2" multiplier="-1" offset="0"/>
  </joint>
  <joint name="brace_fixed_1" type="fixed">
    <parent link="base"/>
    <child link="brace_top"/>
    <origin xyz="0.1 0 0.35" rpy="0 0 0"/>
  </joint>
  <joint name="brace_fixed_2" type="fixed">
    <parent link="brace_top"/>
    <child link="brace_bottom"/>
    <origin xyz="0 0 -0.35" rpy="0 0 0"/>
  </joint>
  <joint name="brace_close" type="fixed">
    <parent link="brace_bottom"/>
    <child link="base"/>
    <origin xyz="-0.1 0 0" rpy="0 0 0"/>
  </joint>
  <vendor:inspection date="2026-09-01" inspector="zhang">
    <note>厂商自定义检查记录，导入必须原样保留</note>
  </vendor:inspection>
</robot>
"""

    val SEED_VALUES = """{"j1":0.1,"j2":0.35}"""

    val SECOND_SERIAL = "SN-DEMO-008"

    val SEED_URDF_BAD: String = """<?xml version="1.0"?>
<robot name="demo_loop_bad">
  <link name="a"/>
  <link name="b"/>
  <link name="c"/>
  <joint name="ab" type="fixed">
    <parent link="a"/><child link="b"/>
    <origin xyz="1 0 0" rpy="0 0 0"/>
  </joint>
  <joint name="bc" type="fixed">
    <parent link="b"/><child link="c"/>
    <origin xyz="0 1 0" rpy="0 0 0"/>
  </joint>
  <joint name="ca" type="fixed">
    <parent link="c"/><child link="a"/>
    <!-- 刻意制造不一致：理论应为 (0 -1 0) -->
    <origin xyz="0 -1.02 0" rpy="0 0 0"/>
  </joint>
</robot>
"""
}
