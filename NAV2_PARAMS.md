# Nav2 Parameters Reference (nav2_params.yaml)

เอกสารอธิบายพารามิเตอร์ทั้งหมดในไฟล์ `nav2_params.yaml` สำหรับหุ่นยนต์แบบ Differential Drive
ที่ใช้ Nav2 (AMCL + NavFn + DWB + Costmap 2D)

> **หมายเหตุเรื่องเวอร์ชัน:** ไฟล์นี้ผสมรูปแบบของหลาย distro
> - `recoveries_server`, `default_bt_xml_filename` → รูปแบบ Foxy/Galactic
> - `nav2_amcl::DifferentialMotionModel`, `nav2_navfn_planner/NavfnPlanner` → รูปแบบ Humble ขึ้นไป
>
> ให้ตรวจชื่อ plugin/พารามิเตอร์ให้ตรงกับ distro ที่ใช้จริงก่อนรัน

## สารบัญ

- [A. ไฟล์นี้ใช้กับอะไร](#a-ไฟล์นี้ใช้กับอะไร)
- [B. ภาพรวมการทำงานของระบบ](#b-ภาพรวมการทำงานของระบบ)
- [C. สิ่งที่ต้องมีก่อนใช้งาน](#c-สิ่งที่ต้องมีก่อนใช้งาน)
- [D. วิธีใช้งานทีละขั้น](#d-วิธีใช้งานทีละขั้น)
- [E. แก้ปัญหาที่พบบ่อย](#e-แก้ปัญหาที่พบบ่อย)

**รายละเอียดพารามิเตอร์:**

1. [ปัญหาที่ควรแก้ก่อน commit](#1-ปัญหาที่ควรแก้ก่อน-commit)
2. [amcl](#2-amcl--การระบุตำแหน่งบนแผนที่)
3. [bt_navigator](#3-bt_navigator--behavior-tree)
4. [controller_server](#4-controller_server--ควบคุมการเคลื่อนที่)
5. [local_costmap](#5-local_costmap)
6. [global_costmap](#6-global_costmap)
7. [map_server / map_saver](#7-map_server--map_saver)
8. [planner_server](#8-planner_server--วางเส้นทางโดยรวม)
9. [recoveries_server](#9-recoveries_server--พฤติกรรมกู้สถานการณ์)
10. [robot_state_publisher](#10-robot_state_publisher)
11. [waypoint_follower](#11-waypoint_follower)
12. [ลำดับการจูนที่แนะนำ](#12-ลำดับการจูนที่แนะนำ)

---

## A. ไฟล์นี้ใช้กับอะไร

`nav2_params.yaml` คือไฟล์ตั้งค่าของ **Nav2 (ROS 2 Navigation Stack)** ซึ่งเป็นชุดซอฟต์แวร์ที่ทำให้
หุ่นยนต์เคลื่อนที่ (mobile robot) **นำทางตัวเองแบบอัตโนมัติ** จากจุด A ไปจุด B โดยหลบสิ่งกีดขวางได้

| ความสามารถ | ทำงานโดย | ตัวอย่างการใช้งาน |
|---|---|---|
| รู้ว่าหุ่นอยู่ตรงไหนบนแผนที่ | `amcl` | หุ่นในโกดัง/อาคารที่มีแผนที่อยู่แล้ว |
| วางเส้นทางจากจุดปัจจุบันไปเป้าหมาย | `planner_server` + `global_costmap` | กดเป้าหมายใน RViz แล้วหุ่นหาทางเอง |
| ขับตามเส้นทางและหลบสิ่งกีดขวางสด ๆ | `controller_server` (DWB) + `local_costmap` | คนเดินตัดหน้า กล่องวางขวางทาง |
| ตัดสินใจเมื่อติด (หมุน ถอย รอ แล้วลองใหม่) | `bt_navigator` + `recoveries_server` | หุ่นติดมุมแล้วออกเองได้ |
| วิ่งผ่านหลายจุดต่อเนื่อง | `waypoint_follower` | ลาดตระเวน ส่งของหลายจุด |

**เหมาะกับ:** หุ่นล้อแบบ Differential Drive (2 ล้อขับ + ล้อพยุง) ที่มี LiDAR 2D และ odometry
ทั้งในโปรแกรมจำลอง (Gazebo) และหุ่นจริง

**ไม่ได้ทำ:** สร้างแผนที่ (ต้องใช้ SLAM เช่น `slam_toolbox` แยกต่างหาก) และไม่ได้ขับมอเตอร์โดยตรง
(Nav2 แค่ publish `/cmd_vel` ต้องมี driver ของหุ่นรับไปสั่งมอเตอร์ต่อ)

---

## B. ภาพรวมการทำงานของระบบ

```
                    ┌──────────────────────────────┐
   เป้าหมาย ──────► │        bt_navigator          │ ◄── Behavior Tree (ตัดสินใจ)
 (RViz / action)    └───┬──────────┬──────────┬────┘
                        │          │          │
                        ▼          ▼          ▼
               planner_server  controller_server  recoveries_server
               (NavFn)         (DWB)              (spin/backup/wait)
                    │               │                    │
              global_costmap   local_costmap             │
                    ▲               ▲                    │
                    │               │                    │
        map_server ─┘      /scan ───┴──── amcl ◄── /scan + /odom
        (map.yaml)                         │
                                    TF: map → odom → base_link
                                           │
        controller_server ──► /cmd_vel ──► driver ของหุ่น ──► มอเตอร์
```

**ข้อมูลเข้า (Inputs)**

| ข้อมูล | Topic / แหล่งที่มา | ใช้โดย |
|---|---|---|
| LaserScan | `/scan` | amcl, costmap ทั้งสอง |
| Odometry | `/odom` และ TF `odom → base_link` | amcl, controller, bt_navigator |
| แผนที่ | ไฟล์ `map.yaml` + `.pgm/.png` | map_server → amcl, global_costmap |
| ตำแหน่งเริ่มต้น | `/initialpose` (กดใน RViz) | amcl |
| เป้าหมาย | action `/navigate_to_pose` | bt_navigator |

**ข้อมูลออก (Outputs):** `/cmd_vel` (`geometry_msgs/Twist`) และ TF `map → odom`

**ลำดับการทำงานเมื่อสั่งไปเป้าหมาย**

1. `bt_navigator` รับ goal แล้วเริ่มรัน Behavior Tree
2. `planner_server` วางเส้นทางบน global costmap
3. `controller_server` (DWB) ติดตามเส้นทาง สร้าง `/cmd_vel` ตามข้อมูล local costmap
4. `amcl` ปรับตำแหน่งหุ่นบนแผนที่ตลอดทาง
5. ถ้าหุ่นติด (`progress_checker` ตรวจไม่พบความคืบหน้า) → BT เรียก recovery แล้ววางแผนใหม่
6. เมื่อเข้าเกณฑ์ `goal_checker` → สำเร็จ

---

## C. สิ่งที่ต้องมีก่อนใช้งาน

### ซอฟต์แวร์

```bash
# ตัวอย่าง (แทน <distro> ด้วย foxy / galactic / humble ที่ใช้จริง)
sudo apt install ros-<distro>-navigation2 ros-<distro>-nav2-bringup
sudo apt install ros-<distro>-slam-toolbox        # ถ้าต้องสร้างแผนที่
sudo apt install ros-<distro>-teleop-twist-keyboard
```

### ฮาร์ดแวร์/ระบบหุ่น (ต้องเช็คให้ครบก่อน เพราะเป็นสาเหตุหลักที่ Nav2 ไม่ทำงาน)

| รายการ | ต้องเป็นแบบนี้ | ตรวจด้วย |
|---|---|---|
| LiDAR 2D | publish `sensor_msgs/LaserScan` ที่ `/scan` | `ros2 topic hz /scan` |
| Odometry | publish `/odom` และ TF `odom → base_link` (หรือ `base_footprint`) | `ros2 topic echo /odom --once` |
| TF tree | `map → odom → base_link → laser` ต่อกันครบ | `ros2 run tf2_tools view_frames` |
| รับคำสั่งขับ | driver subscribe `/cmd_vel` แล้วสั่งมอเตอร์ | `ros2 topic pub /cmd_vel ...` ทดสอบ |
| แผนที่ | ไฟล์ `map.yaml` + ภาพแผนที่ | ดูหัวข้อ D.1 ถ้ายังไม่มี |

> `map → odom` จะมาจาก AMCL เอง ไม่ต้องทำเอง แต่ `odom → base_link` และ `base_link → laser` ต้องมีจาก driver/URDF

---

## D. วิธีใช้งานทีละขั้น

### D.1 สร้างแผนที่ (ทำครั้งเดียวต่อสถานที่)

```bash
# Terminal 1: เปิดหุ่น/จำลอง (ต้องมี /scan และ /odom)
# Terminal 2: เปิด SLAM
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false

# Terminal 3: ขับหุ่นสำรวจให้ทั่วพื้นที่ (ช้า ๆ)
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# เมื่อแผนที่ครบ บันทึกแผนที่
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

จะได้ `my_map.yaml` และ `my_map.pgm` ใช้ในขั้นถัดไป (ค่า threshold ในการบันทึกมาจาก `map_saver` ในไฟล์ params)

### D.2 วางไฟล์ใน package ของคุณ

```
my_robot_nav/
├── config/
│   └── nav2_params.yaml        ← ไฟล์นี้
├── maps/
│   ├── my_map.yaml
│   └── my_map.pgm
├── launch/
│   └── navigation.launch.py
├── NAV2_PARAMS.md              ← เอกสารนี้
├── CMakeLists.txt  /  package.xml
```

ใน `CMakeLists.txt` ต้องติดตั้งโฟลเดอร์ด้วย:

```cmake
install(DIRECTORY config maps launch DESTINATION share/${PROJECT_NAME})
```

### D.3 เปิดระบบนำทาง

**แบบสั้น (ใช้ launch สำเร็จรูปของ Nav2):**

```bash
ros2 launch nav2_bringup bringup_launch.py \
  map:=$HOME/maps/my_map.yaml \
  params_file:=$HOME/my_ws/src/my_robot_nav/config/nav2_params.yaml \
  use_sim_time:=false \
  autostart:=true
```

- หุ่นจริง → `use_sim_time:=false` / Gazebo → `use_sim_time:=true`
- ค่า `use_sim_time` และ `yaml_filename` จาก argument จะ **ทับ** ค่าในไฟล์ params ให้อัตโนมัติ (ผ่าน `RewrittenYaml`)
  แต่ node ที่ไม่อยู่ในกลุ่มที่ถูกทับ (เช่นบรรทัดที่ตั้ง `True` ค้างไว้ใน `global_costmap`) ควรแก้ในไฟล์ให้ตรงกันไว้ก่อน

**แบบ launch file ของตัวเอง** (`launch/navigation.launch.py`):

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory('my_robot_nav')
    nav2 = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'map': os.path.join(pkg, 'maps', 'my_map.yaml'),
                'params_file': os.path.join(pkg, 'config', 'nav2_params.yaml'),
                'use_sim_time': use_sim_time,
                'autostart': 'true',
            }.items(),
        ),
    ])
```

```bash
cd ~/my_ws && colcon build --packages-select my_robot_nav && source install/setup.bash
ros2 launch my_robot_nav navigation.launch.py use_sim_time:=false
```

### D.4 เปิด RViz และตั้งตำแหน่งเริ่มต้น

```bash
ros2 launch nav2_bringup rviz_launch.py
```

1. กดปุ่ม **2D Pose Estimate** ที่แถบเครื่องมือ
2. คลิกตำแหน่งจริงของหุ่นบนแผนที่ แล้วลากเพื่อกำหนดทิศหัวหุ่น
3. ดูว่าจุด LiDAR ทับกับผนังในแผนที่ และกลุ่มอนุภาคสีเขียว (AMCL) ลู่เข้ารอบหุ่น
4. ถ้าไม่ตรง ให้กดตั้งใหม่ หรือขับหุ่นหมุนเบา ๆ ให้ AMCL ปรับตัว

### D.5 สั่งให้หุ่นไปเป้าหมาย

**วิธี 1: RViz** → ปุ่ม **Nav2 Goal** → คลิกปลายทางบนแผนที่ แล้วลากเพื่อกำหนดทิศ

**วิธี 2: Command line**

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
"{pose: {header: {frame_id: map}, pose: {position: {x: 1.5, y: 0.5, z: 0.0}, orientation: {w: 1.0}}}}" \
--feedback
```

**วิธี 3: หลายจุดต่อเนื่อง (`waypoint_follower`)** → ใช้ปุ่ม **Waypoint mode** ใน RViz plugin ของ Nav2
แล้วคลิกหลายจุด จากนั้นกด **Start Navigation** หรือส่ง action `/follow_waypoints`

**วิธี 4: จากโค้ด Python** (ใช้ `nav2_simple_commander`)

```python
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator

rclpy.init()
nav = BasicNavigator()
nav.waitUntilNav2Active()

goal = PoseStamped()
goal.header.frame_id = 'map'
goal.header.stamp = nav.get_clock().now().to_msg()
goal.pose.position.x = 1.5
goal.pose.position.y = 0.5
goal.pose.orientation.w = 1.0

nav.goToPose(goal)
while not nav.isTaskComplete():
    pass  # อ่าน nav.getFeedback() ได้ที่นี่
print(nav.getResult())
```

> `nav2_simple_commander` มีใน Galactic ขึ้นไป (Foxy ไม่มี)

### D.6 ตรวจว่าระบบพร้อม

```bash
# lifecycle ของแต่ละ node ต้องเป็น active
ros2 lifecycle get /amcl
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator

# ดู node และ topic ที่เกี่ยวข้อง
ros2 node list
ros2 topic list | grep -E "scan|odom|cmd_vel|costmap|plan"
```

### D.7 ปรับค่าระหว่างรัน (ไม่ต้องรีสตาร์ท)

```bash
ros2 param get /controller_server FollowPath.max_vel_x
ros2 param set /controller_server FollowPath.max_vel_x 0.2
ros2 param dump /controller_server > controller_tuned.yaml   # เก็บค่าที่จูนแล้ว
```

ค่าที่ `param set` เป็นค่าชั่วคราว ถ้าพอใจแล้วต้องแก้กลับเข้าไฟล์ `nav2_params.yaml` ด้วย

---

## E. แก้ปัญหาที่พบบ่อย

| อาการ | สาเหตุที่เป็นไปได้ | วิธีตรวจ/แก้ |
|---|---|---|
| Node ไม่ขึ้น `active` / launch ค้าง | YAML ผิดไวยากรณ์ (ดูหัวข้อ 1) | `python3 -c "import yaml; yaml.safe_load(open('nav2_params.yaml'))"` |
| `Timed out waiting for transform` / `extrapolation into the future` | `use_sim_time` ไม่ตรงกัน หรือ TF ขาด | ตั้ง `use_sim_time` ให้ตรงทุก node ; เพิ่ม `transform_tolerance` ; `view_frames` |
| `Invalid frame ID "base_link"` | base frame ใน params ไม่ตรงกับ TF จริง | เลือก `base_link` หรือ `base_footprint` ให้ตรงกันทั้งไฟล์ |
| หุ่นไม่ขยับ ทั้งที่มีเส้นทาง | ไม่มี driver รับ `/cmd_vel` หรือยังไม่ได้ตั้ง 2D Pose Estimate | `ros2 topic echo /cmd_vel` ; ตั้งตำแหน่งเริ่มต้น |
| หุ่นหมุนวนไม่เข้า goal | `yaw_goal_tolerance` เล็กเกิน หรือ `RotateToGoal` แรงเกิน | เพิ่ม tolerance ; ลด `RotateToGoal.scale` |
| หุ่นแกว่งซ้าย-ขวา/กระตุก | `controller_frequency` ต่ำ (5 Hz) | เพิ่มเป็น 10–20 Hz |
| ชนสิ่งกีดขวาง/เฉียดมาก | `inflation_radius` เล็ก หรือ `BaseObstacle.scale` ต่ำ | เพิ่ม `inflation_radius`, `BaseObstacle.scale` |
| ผ่านช่องแคบไม่ได้ | `inflation_radius` + `robot_radius` ใหญ่เกินช่อง | ลด `inflation_radius` ; ตรวจ `robot_radius` ตรงขนาดจริง |
| ตำแหน่งบนแผนที่เพี้ยน/กระโดด | odometry ไม่แม่น หรือ `alpha*` ต่ำเกิน | ปรับ `alpha1–4` ; ตรวจการสอบเทียบล้อ |
| วางแผนไม่ได้ (`Failed to create a plan`) | goal อยู่ในสิ่งกีดขวาง/นอกแผนที่ หรือ global costmap ไม่มีข้อมูล | เลือก goal ที่โล่ง ; ดู global costmap ใน RViz |
| CPU สูง | `voxel_layer` ซ้ำซ้อน, `max_particles` สูง, Groot/debug เปิดอยู่ | ตัด voxel ; ลด particles ; ปิด debug |

---

## 1. ปัญหาที่ควรแก้ก่อน commit

### ❌ Error ที่ทำให้ YAML parse ไม่ผ่าน

| ตำแหน่ง | ปัญหา | วิธีแก้ |
|---|---|---|
| `FollowPath` ก่อน `linear_granularity` | บรรทัด `Linear resolution` ไม่มี `#` นำหน้า | แก้เป็น `# Linear resolution` |
| ก่อน `planner_server` | มีอักขระ `•` หลุดอยู่ใต้ `map_saver` | ลบทิ้ง |
| หลายบรรทัดว่าง | มี zero-width space (U+200B) ปนอยู่ | ลบด้วย `sed -i 's/\xE2\x80\x8B//g' nav2_params.yaml` |
| ทุก key | เขียน `key :` (เว้นวรรคก่อน colon) | YAML รับได้ แต่ควรเขียน `key:` ให้เป็นมาตรฐาน |

### ⚠️ ค่าที่ขัดแย้งกันเอง (ควรตรวจ)

| เรื่อง | รายละเอียด | คำแนะนำ |
|---|---|---|
| `use_sim_time` | `global_costmap` ตั้ง `True` แต่ node อื่น `False` | ทุก node ต้องตรงกัน (Gazebo = `True`, หุ่นจริง = `False`) |
| Base frame | `amcl` ใช้ `base_footprint` แต่ `bt_navigator`/costmap/recoveries ใช้ `base_link` | เลือกตัวเดียวให้ตรงกับ TF tree ของหุ่น |
| ความเร็ว | `max_vel_x: 0.35` แต่ `max_speed_xy: 0.22` | ค่า 0.22 คือเพดานจริง ทำให้ 0.35 ไม่มีผล ปรับให้สอดคล้อง |
| `min_y_velocity_threshold` | `0.5` | หุ่น diff-drive ไม่มีแกน Y ควรเป็น `0.001` เหมือนแกน X |
| `waypoint_follower.loop_rate` | `2000` | ค่า default คือ `20` (Hz) ค่า 2000 สูงผิดปกติ น่าจะพิมพ์เกิน |
| `robot_radius` | local = `0.15`, global = `0.2` | ควรใช้ค่าเดียวกันถ้าเป็นหุ่นตัวเดียวกัน |
| `local_costmap.plugins` | ใช้ทั้ง `obstacle_layer` และ `voxel_layer` กับ `/scan` เดียวกัน | LaserScan 2D ใช้ `obstacle_layer` อย่างเดียวพอ ลดภาระ CPU |
| `local_costmap.static_layer` | ตั้งค่าไว้แต่ไม่อยู่ใน `plugins` | ไม่มีผล (ลบหรือเพิ่มเข้า plugins ตามต้องการ) |
| `z_hit+z_max+z_rand+z_short` | รวม = 1.1 | โมเดล `likelihood_field` ใช้แค่ `z_hit` กับ `z_rand` จึงไม่กระทบ แต่ควรให้รวม ≈ 1 |
| `track_unknown_space: false` + `allow_unknown: true` | global costmap ไม่ติดตามพื้นที่ไม่รู้ แต่ planner อนุญาตผ่าน | ถ้าแผนที่มี unknown ให้ตั้ง `track_unknown_space: true` |
| คำอธิบาย `alpha1–alpha5` ในไฟล์เดิม | คอมเมนต์เดิมสับสน/ผิดความหมาย | ดูตารางที่ถูกต้องในหัวข้อ AMCL |
| `tf_publish_rate` (amcl) | ไม่ใช่พารามิเตอร์มาตรฐานของ nav2_amcl ในหลายเวอร์ชัน | ตรวจกับ `ros2 param list /amcl` ถ้าไม่มีให้ลบ |

---

## 2. `amcl` — การระบุตำแหน่งบนแผนที่

AMCL ใช้ Particle Filter (อนุภาคแต่ละตัว = สมมติฐานตำแหน่งหุ่น) ร่วมกับ LaserScan และ odometry
เพื่อประมาณตำแหน่งหุ่นในเฟรม `map` และ publish TF `map → odom`

### 2.1 พื้นฐานและเฟรม

| พารามิเตอร์ | ค่า | หน่วย | ความหมาย |
|---|---|---|---|
| `use_sim_time` | `False` | bool | `True` = ใช้เวลาจาก `/clock` (จำลอง) `False` = ใช้เวลาจริง |
| `base_frame_id` | `base_footprint` | string | เฟรมฐานของหุ่น ต้องมีใน TF tree |
| `odom_frame_id` | `odom` | string | เฟรม odometry |
| `global_frame_id` | `map` | string | เฟรมแผนที่ (AMCL publish `map→odom`) |
| `scan_topic` | `scan` | string | topic ของ LaserScan |
| `tf_broadcast` | `true` | bool | `false` = ไม่ publish TF `map→odom` (ใช้เมื่อมีตัวอื่นทำแทน) |
| `transform_tolerance` | `0.2` | วินาที | ระยะเวลาที่ TF ถูก post-date ไปข้างหน้า เพิ่มถ้าเจอ TF extrapolation error |
| `save_pose_rate` | `0.5` | Hz | ความถี่การบันทึกตำแหน่งล่าสุดลง parameter server |

### 2.2 Odometry motion model (`alpha1`–`alpha5`)

ค่าเหล่านี้บอกว่า "เชื่อ odometry แค่ไหน" ยิ่งสูง = ถือว่า odom มี noise มาก อนุภาคจะกระจายมากขึ้น

| พารามิเตอร์ | ค่า | noise ของอะไร |
|---|---|---|
| `alpha1` | `0.2` | การหมุน (rotation) ที่เกิดจากการหมุน |
| `alpha2` | `0.2` | การหมุน ที่เกิดจากการเคลื่อนที่ตรง |
| `alpha3` | `0.2` | การเคลื่อนที่ตรง ที่เกิดจากการเคลื่อนที่ตรง |
| `alpha4` | `0.2` | การเคลื่อนที่ตรง ที่เกิดจากการหมุน |
| `alpha5` | `0.2` | การเคลื่อนที่ด้านข้าง (ใช้เฉพาะโมเดล Omni — diff-drive ไม่ใช้) |
| `robot_model_type` | `nav2_amcl::DifferentialMotionModel` | โมเดลการเคลื่อนที่: Differential / Omnidirectional |

**จูน:** ล้อลื่นมาก/odom แย่ → เพิ่ม alpha1–4 ; odom แม่นมาก → ลดลง (เช่น 0.05–0.1)

### 2.3 จำนวนอนุภาคและ KLD sampling

| พารามิเตอร์ | ค่า | ความหมาย |
|---|---|---|
| `min_particles` | `500` | จำนวนอนุภาคต่ำสุด (ตอนมั่นใจตำแหน่งแล้ว) |
| `max_particles` | `2000` | จำนวนอนุภาคสูงสุด (ตอนไม่แน่ใจตำแหน่ง) เพิ่ม = แม่นขึ้นแต่กิน CPU |
| `pf_err` | `0.05` | ความคลาดเคลื่อนสูงสุดระหว่าง distribution จริงกับที่ประมาณ (KLD) |
| `pf_z` | `0.99` | ค่าความเชื่อมั่น (quantile) สำหรับ `pf_err` |
| `resample_interval` | `1` | resample ทุกกี่ครั้งที่มีการ update filter |

### 2.4 กลไกกู้คืน (Recovery)

| พารามิเตอร์ | ค่า | ความหมาย |
|---|---|---|
| `recovery_alpha_slow` | `0.0` | อัตราการเฉลี่ยแบบช้าของน้ำหนักอนุภาค `0.0` = **ปิด** |
| `recovery_alpha_fast` | `0.0` | อัตราการเฉลี่ยแบบเร็ว `0.0` = **ปิด** |

ถ้าต้องการให้ AMCL สุ่มอนุภาคใหม่เมื่อหลงตำแหน่ง ลองตั้ง `slow ≈ 0.001`, `fast ≈ 0.1`

### 2.5 เงื่อนไขการอัปเดต

| พารามิเตอร์ | ค่า | หน่วย | ความหมาย |
|---|---|---|---|
| `update_min_d` | `0.25` | เมตร | ต้องเคลื่อนที่ได้อย่างน้อยเท่านี้จึง update filter |
| `update_min_a` | `0.2` | เรเดียน (~11°) | ต้องหมุนอย่างน้อยเท่านี้จึง update filter |

### 2.6 Laser model

| พารามิเตอร์ | ค่า | ความหมาย |
|---|---|---|
| `laser_model_type` | `likelihood_field` | `beam` / `likelihood_field` / `likelihood_field_prob` (เร็วและทนต่อ noise กว่า beam) |
| `max_beams` | `60` | จำนวน beam ที่ใช้ต่อการ update (สุ่มกระจายทั่ว scan) |
| `laser_min_range` | `-1.0` | `-1` = ใช้ค่าจาก LaserScan msg |
| `laser_max_range` | `100.0` | เมตร — ระยะไกลสุดที่ใช้ (`-1` = ตาม msg) |
| `laser_likelihood_max_dist` | `2.0` | เมตร — ระยะ inflate สิ่งกีดขวางสำหรับ likelihood field |
| `sigma_hit` | `0.2` | เมตร — ส่วนเบี่ยงเบนมาตรฐานของ Gaussian สำหรับ "ชนถูกตำแหน่ง" |
| `z_hit` | `0.5` | น้ำหนักการวัดที่ตรงกับแผนที่ |
| `z_rand` | `0.5` | น้ำหนักการวัดแบบสุ่ม/noise |
| `z_max` | `0.05` | น้ำหนักการวัดที่ได้ max range (ใช้กับ `beam` เท่านั้น) |
| `z_short` | `0.05` | น้ำหนักการวัดสั้นผิดปกติ (ใช้กับ `beam` เท่านั้น) |
| `lambda_short` | `0.1` | ค่า decay ของ short reading (ใช้กับ `beam` เท่านั้น) |

### 2.7 Beam skipping

| พารามิเตอร์ | ค่า | ความหมาย |
|---|---|---|
| `do_beamskip` | `false` | เปิด/ปิดการข้าม beam ที่ไม่สอดคล้องกับแผนที่ (มีประโยชน์เมื่อมีคนเดินผ่าน) |
| `beam_skip_distance` | `0.5` | เมตร — ระยะที่ถือว่า beam ไม่ตรงกับแผนที่ |
| `beam_skip_threshold` | `0.3` | สัดส่วน beam ที่ต้องตรงจึงจะไม่ข้าม |
| `beam_skip_error_threshold` | `0.9` | ถ้า beam ที่ถูกข้ามเกินสัดส่วนนี้ จะ reset การข้าม (กัน filter เพี้ยน) |

### 2.8 `amcl_map_client`, `amcl_rclcpp_node`

โหนดช่วยเหลือภายใน (client โหลดแผนที่ / node สำหรับ service) มีเพียง `use_sim_time`

---

## 3. `bt_navigator` — Behavior Tree

ตัวประสานงานระดับสูง: รับ goal → เรียก planner/controller/recovery ตาม Behavior Tree

| พารามิเตอร์ | ค่า | ความหมาย |
|---|---|---|
| `use_sim_time` | `False` | ดูหัวข้อ AMCL |
| `global_frame` | `map` | เฟรมอ้างอิงของ goal |
| `robot_base_frame` | `base_link` | เฟรมฐานหุ่น (ควรตรงกับ AMCL) |
| `odom_topic` | `/odom` | topic odometry ที่ BT ใช้ติดตามความเร็ว |
| `default_bt_xml_filename` | `navigate_w_replanning_and_recovery.xml` | BT XML เริ่มต้น: วางแผนใหม่ + กู้สถานการณ์เมื่อติด |
| `bt_loop_duration` | `10` | มิลลิวินาที — คาบของ BT loop |
| `default_server_timeout` | `20` | มิลลิวินาที — เวลารอ action server ตอบ |
| `enable_groot_monitoring` | `True` | เปิดการมอนิเตอร์ BT ผ่าน Groot (ปิดตอน production เพื่อลด overhead) |
| `groot_zmq_publisher_port` | `1666` | พอร์ต ZMQ publisher ให้ Groot |
| `groot_zmq_server_port` | `1667` | พอร์ต ZMQ server ให้ Groot |
| `plugin_lib_names` | (รายการ) | ไลบรารี BT node ที่ต้องโหลดเพื่อให้ XML ใช้ node เหล่านั้นได้ |

**กลุ่ม BT plugin หลัก**

| กลุ่ม | ตัวอย่าง |
|---|---|
| Action | `compute_path_to_pose`, `follow_path`, `back_up`, `spin`, `wait`, `navigate_to_pose`, `navigate_through_poses` |
| Condition | `is_stuck`, `goal_reached`, `goal_updated`, `initial_pose_received`, `transform_available`, `time_expired`, `distance_traveled`, `is_battery_low` |
| Control/Decorator | `pipeline_sequence`, `round_robin`, `recovery_node`, `rate_controller`, `distance_controller`, `speed_controller`, `single_trigger` |
| Service | `clear_costmap`, `reinitialize_global_localization` |
| Selector | `planner_selector`, `controller_selector`, `goal_checker_selector` |

**หมายเหตุ:** ถ้า BT XML ที่ใช้ไม่เรียกใช้ node ใด ก็ลบ plugin นั้นออกได้เพื่อลดเวลาเริ่มระบบ

---

## 4. `controller_server` — ควบคุมการเคลื่อนที่

รับเส้นทางจาก planner แล้วสั่ง `cmd_vel` ตามเส้นทาง ผ่าน local planner (DWB)

### 4.1 พารามิเตอร์ของ server

| พารามิเตอร์ | ค่า | ความหมาย |
|---|---|---|
| `controller_frequency` | `5.0` Hz | ความถี่สั่ง cmd_vel (5 ค่อนข้างต่ำ แนะนำ 10–20 Hz เพื่อความนุ่มนวล) |
| `min_x_velocity_threshold` | `0.001` | ความเร็วเชิงเส้นต่ำกว่านี้ถือเป็น 0 (ตัด noise odom) |
| `min_y_velocity_threshold` | `0.5` | ดูหัวข้อ "ปัญหา" — ควรเป็น `0.001` |
| `min_theta_velocity_threshold` | `0.001` | ความเร็วเชิงมุมต่ำกว่านี้ถือเป็น 0 |
| `failure_tolerance` | `0.3` | วินาที — ถ้าคำนวณคำสั่งควบคุมไม่ได้นานกว่านี้ จะถือว่าล้มเหลว |
| `progress_checker_plugin` | `progress_checker` | ชื่ออินสแตนซ์ของตัวตรวจความคืบหน้า |
| `goal_checker_plugins` | `["general_goal_checker"]` | รายชื่อตัวตรวจว่าถึงเป้าหมายแล้ว |
| `controller_plugins` | `["FollowPath"]` | รายชื่อ controller |

### 4.2 `progress_checker` (ตรวจว่าหุ่นขยับจริงหรือติดอยู่)

| พารามิเตอร์ | ค่า | ความหมาย |
|---|---|---|
| `plugin` | `nav2_controller::SimpleProgressChecker` | ชนิด plugin |
| `required_movement_radius` | `0.5` | เมตร — ต้องขยับอย่างน้อยเท่านี้ภายในเวลาที่กำหนด |
| `movement_time_allowance` | `10.0` | วินาที — ถ้าไม่ขยับตามเกณฑ์ → ถือว่าไม่คืบหน้า |

### 4.3 `general_goal_checker` (ตัดสินว่า "ถึงแล้ว")

| พารามิเตอร์ | ค่า | ความหมาย |
|---|---|---|
| `plugin` | `nav2_controller::SimpleGoalChecker` | ชนิด plugin |
| `stateful` | `True` | เมื่อเข้าเกณฑ์ตำแหน่งแล้ว จะจำสถานะ แล้วค่อยเช็คเฉพาะมุม (กันหุ่นแกว่ง) |
| `xy_goal_tolerance` | `0.15` | เมตร — ระยะคลาดเคลื่อนตำแหน่งที่ยอมรับ |
| `yaw_goal_tolerance` | `0.15` | เรเดียน (~8.6°) — มุมคลาดเคลื่อนที่ยอมรับ |

### 4.4 `FollowPath` — DWB Local Planner

DWB สุ่มตัวอย่างคำสั่งความเร็ว → จำลองวิถีล่วงหน้า → ให้คะแนนด้วย critics → เลือกที่ดีที่สุด

**ขีดจำกัดความเร็ว/ความเร่ง**

| พารามิเตอร์ | ค่า | หน่วย | ความหมาย |
|---|---|---|---|
| `min_vel_x` / `max_vel_x` | `0.0` / `0.35` | m/s | ความเร็วเดินหน้าต่ำสุด/สูงสุด (0 = ไม่ถอยหลัง) |
| `min_vel_y` / `max_vel_y` | `0.0` / `0.0` | m/s | ด้านข้าง (diff-drive = 0) |
| `max_vel_theta` | `1.0` | rad/s | ความเร็วหมุนสูงสุด |
| `min_speed_xy` / `max_speed_xy` | `0.0` / `0.22` | m/s | ขอบเขตความเร็วรวมในระนาบ XY |
| `min_speed_theta` | `0.0` | rad/s | ความเร็วหมุนต่ำสุด |
| `acc_lim_x` / `acc_lim_y` / `acc_lim_theta` | `2.5` / `0.0` / `3.2` | m/s² , rad/s² | ความเร่งสูงสุด |
| `decel_lim_x` / `decel_lim_y` / `decel_lim_theta` | `-2.5` / `0.0` / `-3.2` | m/s² , rad/s² | ความหน่วงสูงสุด (ค่าติดลบ) |

**การสุ่มตัวอย่างและจำลองวิถี**

| พารามิเตอร์ | ค่า | ความหมาย |
|---|---|---|
| `vx_samples` | `20` | จำนวนตัวอย่างความเร็วแกน X |
| `vy_samples` | `0` | จำนวนตัวอย่างแกน Y (diff-drive ใช้ 0; DWB ต้อง ≥1 ถ้าเปิดแกน Y) |
| `vtheta_samples` | `40` | จำนวนตัวอย่างความเร็วหมุน |
| `sim_time` | `1.5` | วินาที — เวลาที่จำลองวิถีล่วงหน้า ยาว = มองไกลแต่ช้า |
| `linear_granularity` | `0.05` | เมตร — ระยะห่างระหว่างจุดตรวจตามวิถี |
| `angular_granularity` | `0.025` | เรเดียน — ความละเอียดมุมของจุดตรวจ |
| `transform_tolerance` | `0.2` | วินาที — ความคลาดเคลื่อนเวลา TF |
| `xy_goal_tolerance` | `0.05` | เมตร — ใช้ภายใน DWB/critics (ผลสำเร็จจริงตัดสินโดย goal_checker) |
| `trans_stopped_velocity` | `0.25` | m/s — ต่ำกว่านี้ถือว่าหุ่นหยุดแล้ว (ใช้กับ RotateToGoal) |
| `short_circuit_trajectory_evaluation` | `True` | หยุดประเมินวิถีทันทีเมื่อคะแนนแย่กว่าที่ดีที่สุดแล้ว (เร็วขึ้น) |
| `stateful` | `True` | ให้ critic บางตัวจำสถานะระหว่างรอบ |
| `debug_trajectory_details` | `True` | publish ข้อมูลดีบักวิถี (ปิดตอนใช้งานจริง) |

**Critics** — ลำดับใน `critics: [...]` คือลำดับการประเมิน

| Critic | scale | หน้าที่ |
|---|---|---|
| `RotateToGoal` | `32.0` | เมื่อถึงตำแหน่งแล้ว ให้หมุนตัวไปทิศ goal (`slowing_factor: 5.0` ชะลอเมื่อใกล้, `lookahead_time: -1.0` ใช้ sim_time) |
| `Oscillation` | – | ลงโทษการแกว่งไปมา (กลับทิศซ้ำ ๆ) |
| `BaseObstacle` | `0.02` | ลงโทษวิถีที่เข้าใกล้สิ่งกีดขวางตาม costmap |
| `GoalAlign` | `24.0` | ให้หัวหุ่นหันตรงทิศไปยัง goal (`forward_point_distance: 0.1`) |
| `PathAlign` | `32.0` | ให้หัวหุ่นขนานกับเส้นทาง (`forward_point_distance: 0.1`) |
| `PathDist` | `32.0` | ให้วิ่งใกล้เส้นทางที่วางไว้ |
| `GoalDist` | `24.0` | ให้ปลายวิถีเข้าใกล้ goal |

**จูน:** ชนง่าย → เพิ่ม `BaseObstacle.scale` ; วิ่งออกนอกเส้นทาง → เพิ่ม `PathDist`/`PathAlign` ; เลี้ยวไม่ยอมหันเข้า goal → เพิ่ม `GoalAlign`/`RotateToGoal`

### 4.5 `controller_server_rclcpp_node`

โหนดภายใน มีเพียง `use_sim_time`

---

## 5. `local_costmap`

แผนที่ต้นทุนรอบตัวหุ่นแบบ rolling window ใช้หลบสิ่งกีดขวางแบบเรียลไทม์ ในเฟรม `odom`

| พารามิเตอร์ | ค่า | ความหมาย |
|---|---|---|
| `update_frequency` | `5.0` Hz | ความถี่อัปเดตข้อมูลเซนเซอร์เข้า costmap |
| `publish_frequency` | `2.0` Hz | ความถี่ publish costmap (สำหรับ RViz) |
| `global_frame` | `odom` | เฟรมอ้างอิง (local ใช้ `odom` เพราะต่อเนื่องไม่กระโดด) |
| `robot_base_frame` | `base_link` | เฟรมฐานหุ่น |
| `use_sim_time` | `False` | ดูหัวข้อ AMCL |
| `rolling_window` | `true` | costmap เคลื่อนตามหุ่น |
| `width` / `height` | `3` / `3` | เมตร — ขนาดหน้าต่าง |
| `resolution` | `0.05` | เมตร/เซลล์ |
| `robot_radius` | `0.15` | เมตร — รัศมีหุ่น (ใช้เป็น footprint วงกลม) |
| `plugins` | `obstacle_layer, voxel_layer, inflation_layer` | เลเยอร์ที่ใช้ (เรียงจากล่างขึ้นบน) |
| `always_send_full_costmap` | `True` | ส่ง costmap เต็มทุกครั้ง แทนส่งเฉพาะส่วนที่เปลี่ยน |

### 5.1 `inflation_layer`

| พารามิเตอร์ | ค่า | ความหมาย |
|---|---|---|
| `plugin` | `nav2_costmap_2d::InflationLayer` | ชนิด plugin |
| `inflation_radius` | `0.3` | เมตร — ขยายต้นทุนรอบสิ่งกีดขวางออกไปเท่านี้ (ควร > robot_radius) |
| `cost_scaling_factor` | `3.0` | ต้นทุนลดลงเร็วตามระยะ: **สูง = ลดเร็ว** (หุ่นกล้าเข้าใกล้) **ต่ำ = ลดช้า** (หุ่นเว้นระยะมาก) |

### 5.2 `obstacle_layer`

| พารามิเตอร์ | ค่า | ความหมาย |
|---|---|---|
| `plugin` | `nav2_costmap_2d::ObstacleLayer` | ชนิด plugin |
| `enabled` | `True` | เปิดใช้เลเยอร์ |
| `observation_sources` | `scan` | ชื่อแหล่งข้อมูลที่ใช้ |
| `scan.topic` | `/scan` | topic เซนเซอร์ |
| `scan.data_type` | `LaserScan` | ชนิดข้อมูล (`LaserScan` / `PointCloud2`) |
| `scan.max_obstacle_height` | `2.0` | เมตร — สูงสุดที่ถือเป็นสิ่งกีดขวาง |
| `scan.marking` | `True` | ใช้ข้อมูลนี้มาร์กสิ่งกีดขวาง |
| `scan.clearing` | `True` | ใช้ raytrace ลบสิ่งกีดขวางที่หายไป |

### 5.3 `voxel_layer` (3D, ต้องการข้อมูลความสูง)

| พารามิเตอร์ | ค่า | ความหมาย |
|---|---|---|
| `publish_voxel_map` | `True` | publish แผนที่ voxel เพื่อดีบัก |
| `origin_z` | `0.0` | เมตร — ความสูงเริ่มต้นของ voxel grid |
| `z_resolution` | `0.05` | เมตร/voxel ในแกน Z |
| `z_voxels` | `16` | จำนวน voxel แกน Z (สูงสุด 16) → ครอบคลุม 0.8 ม. |
| `mark_threshold` | `0` | จำนวน voxel ที่ถูกครอบครองในคอลัมน์เพื่อถือว่าเป็นสิ่งกีดขวาง |
| `max_obstacle_height` | `2.0` | เมตร |
| `scan.raytrace_max_range` / `raytrace_min_range` | `3.0` / `0.0` | เมตร — ช่วงที่ใช้ลบสิ่งกีดขวางด้วย raytrace |
| `scan.obstacle_max_range` / `obstacle_min_range` | `2.5` / `0.0` | เมตร — ช่วงที่ใช้มาร์กสิ่งกีดขวาง |

> ถ้าใช้ LiDAR 2D อย่างเดียว ให้ใช้ `obstacle_layer` และตัด `voxel_layer` ทิ้งได้

### 5.4 `static_layer`

`map_subscribe_transient_local: True` — รับแผนที่แบบ latched (ไม่อยู่ใน `plugins` จึงไม่ทำงาน)

### 5.5 โหนดลูก

`local_costmap_client`, `local_costmap_rclcpp_node` — มีเพียง `use_sim_time`

---

## 6. `global_costmap`

แผนที่ต้นทุนทั้งแผนที่ ในเฟรม `map` ให้ planner วางเส้นทางไกล

| พารามิเตอร์ | ค่า | ความหมาย |
|---|---|---|
| `update_frequency` | `1.0` Hz | ความถี่อัปเดต |
| `publish_frequency` | `1.0` Hz | ความถี่ publish |
| `global_frame` | `map` | เฟรมอ้างอิง |
| `robot_base_frame` | `base_link` | เฟรมฐานหุ่น |
| `use_sim_time` | `True` | ⚠️ ไม่ตรงกับ node อื่น ตรวจสอบ |
| `robot_radius` | `0.2` | เมตร |
| `resolution` | `0.05` | เมตร/เซลล์ |
| `track_unknown_space` | `false` | `true` = แยกพื้นที่ไม่รู้ (unknown) ออกจากพื้นที่ว่าง |
| `plugins` | `static_layer, obstacle_layer, voxel_layer, inflation_layer` | เลเยอร์ที่ใช้ |
| `always_send_full_costmap` | `True` | ส่ง costmap เต็มทุกครั้ง |

เลเยอร์ย่อยมีความหมายเดียวกับ local costmap:
- `static_layer` — ใช้ plugin `nav2_costmap_2d::StaticLayer` อ่านแผนที่จาก map_server
- `obstacle_layer`, `voxel_layer` — เหมือนหัวข้อ 5.2–5.3
- `inflation_layer` — `cost_scaling_factor: 3.0`, `inflation_radius: 0.3`

โหนดลูก `global_costmap_client`, `global_costmap_rclcpp_node` — มีเพียง `use_sim_time`

---

## 7. `map_server` / `map_saver`

### `map_server` — โหลดแผนที่

| พารามิเตอร์ | ค่า | ความหมาย |
|---|---|---|
| `use_sim_time` | `False` | – |
| `yaml_filename` | `map.yaml` | ไฟล์ metadata แผนที่ (path ของภาพ, resolution, origin) |

### `map_saver` — บันทึกแผนที่

| พารามิเตอร์ | ค่า | ความหมาย |
|---|---|---|
| `save_map_timeout` | `5.0` | วินาที — รอรับแผนที่ก่อนบันทึก |
| `free_thresh_default` | `0.25` | ความน่าจะเป็นต่ำกว่านี้ = ช่องว่าง (free) |
| `occupied_thresh_default` | `0.65` | ความน่าจะเป็นสูงกว่านี้ = ช่องถูกครอบครอง (occupied) |
| `map_subscribe_transient_local` | `True` | รับแผนที่แบบ latched |

---

## 8. `planner_server` — วางเส้นทางโดยรวม

| พารามิเตอร์ | ค่า | ความหมาย |
|---|---|---|
| `expected_planner_frequency` | `5.0` Hz | ความถี่วางแผนที่คาดหวัง (เตือนถ้าช้ากว่านี้) |
| `use_sim_time` | `False` | – |
| `planner_plugins` | `["GridBased"]` | รายชื่อ planner |

### `GridBased` (NavFn)

| พารามิเตอร์ | ค่า | ความหมาย |
|---|---|---|
| `plugin` | `nav2_navfn_planner/NavfnPlanner` | ใช้ Dijkstra/A* บน grid |
| `tolerance` | `0.5` | เมตร — ถ้า goal ไปไม่ได้ ยอมวางแผนไปจุดใกล้เคียงในรัศมีนี้ |
| `use_astar` | `false` | `false` = Dijkstra (ให้ผลเสถียร), `true` = A* (เร็วกว่าในบางกรณี) |
| `allow_unknown` | `true` | อนุญาตให้ผ่านพื้นที่ unknown |

โหนดลูก `planner_server_rclcpp_node` — มีเพียง `use_sim_time`

---

## 9. `recoveries_server` — พฤติกรรมกู้สถานการณ์

เรียกใช้เมื่อหุ่นติด/วางแผนไม่ได้ (BT เรียก `spin`, `backup`, `wait`)

| พารามิเตอร์ | ค่า | ความหมาย |
|---|---|---|
| `costmap_topic` | `local_costmap/costmap_raw` | costmap ที่ใช้ตรวจชนระหว่างกู้สถานการณ์ |
| `footprint_topic` | `local_costmap/published_footprint` | footprint หุ่นสำหรับตรวจชน |
| `cycle_frequency` | `5.0` Hz | ความถี่ลูป |
| `recovery_plugins` | `["spin", "backup", "wait"]` | รายการ plugin |
| `spin.plugin` | `nav2_recoveries/Spin` | หมุนตัวกับที่ |
| `backup.plugin` | `nav2_recoveries/BackUp` | ถอยหลัง |
| `wait.plugin` | `nav2_recoveries/Wait` | รอเฉย ๆ |
| `global_frame` | `odom` | เฟรมอ้างอิง |
| `robot_base_frame` | `base_link` | เฟรมฐานหุ่น |
| `transform_timeout` | `0.1` | วินาที — รอ TF |
| `simulate_ahead_time` | `2.0` | วินาที — จำลองล่วงหน้าเพื่อตรวจว่าจะชนไหม |
| `max_rotational_vel` | `1.0` | rad/s |
| `min_rotational_vel` | `0.4` | rad/s — ต่ำกว่านี้มอเตอร์อาจไม่หมุน |
| `rotational_acc_lim` | `3.2` | rad/s² |

> ใน Humble ขึ้นไป `recoveries_server` เปลี่ยนชื่อเป็น `behavior_server`

---

## 10. `robot_state_publisher`

แปลง URDF → TF ของลิงก์ต่าง ๆ มีเพียง `use_sim_time`

---

## 11. `waypoint_follower`

ให้หุ่นวิ่งผ่านหลายจุดต่อเนื่อง

| พารามิเตอร์ | ค่า | ความหมาย |
|---|---|---|
| `loop_rate` | `2000` | Hz — ความถี่ตรวจสถานะ ⚠️ ค่า default `20` ตรวจว่าตั้งใจหรือพิมพ์เกิน |
| `stop_on_failure` | `false` | `false` = ถ้าจุดใดล้มเหลวให้ข้ามไปจุดถัดไป |
| `waypoint_task_executor_plugin` | `wait_at_waypoint` | งานที่ทำเมื่อถึงแต่ละจุด |
| `wait_at_waypoint.plugin` | `nav2_waypoint_follower::WaitAtWaypoint` | plugin รอที่จุด |
| `wait_at_waypoint.enabled` | `True` | เปิดใช้งาน |
| `wait_at_waypoint.waypoint_pause_duration` | `200` | มิลลิวินาที — เวลาหยุดรอที่แต่ละจุด |

---

## 12. ลำดับการจูนที่แนะนำ

1. **แก้ YAML ให้ parse ผ่านก่อน** (หัวข้อ 1) แล้วเช็คด้วย `python3 -c "import yaml; yaml.safe_load(open('nav2_params.yaml'))"`
2. **TF และเวลา:** ให้ `use_sim_time` และ base frame สอดคล้องทั้งระบบ
3. **Localization (AMCL):** ปักตำแหน่งเริ่มต้นใน RViz → ขับช้า ๆ ดูว่าอนุภาคลู่เข้า ปรับ `alpha*`, `sigma_hit`
4. **Costmap:** ดู local/global costmap ใน RViz ปรับ `robot_radius`, `inflation_radius`, `cost_scaling_factor`
5. **Planner:** ตรวจเส้นทางโดยรวมให้สมเหตุสมผล
6. **Controller (DWB):** เพิ่ม `controller_frequency` เป็น 10–20 Hz แล้วจูน critics ทีละตัว
7. **Recovery/BT:** ทดสอบกรณีหุ่นติด ตรวจว่า spin/backup/wait ทำงาน
8. **ปิดของดีบัก** (`debug_trajectory_details`, `enable_groot_monitoring`) ก่อน deploy

## คำสั่งตรวจสอบที่มีประโยชน์

```bash
# ดูพารามิเตอร์ที่ node ใช้งานจริง
ros2 param list /amcl
ros2 param get /controller_server FollowPath.max_vel_x

# ตรวจ TF tree
ros2 run tf2_tools view_frames

# เปลี่ยนค่าขณะรัน (ทดลองจูนโดยไม่ต้องรีสตาร์ท)
ros2 param set /controller_server FollowPath.PathAlign.scale 40.0
```
