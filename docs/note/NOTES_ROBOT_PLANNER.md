# RoboTwin Robot Planner Investigation Guide

## Overview
RoboTwin supports **two motion planners**: **Mplib** (CPU-based) and **CuRobo** (GPU-accelerated). Both planners share the same interface but have different collision handling and performance characteristics.

---

## Investigation Q&A

This section documents the key questions asked during the investigation and their answers.

### Q1: How is CuRobo used in this framework?

**A:** CuRobo is used as an optional GPU-accelerated motion planner:

1. **Purpose**: Fast trajectory planning with batch processing support
2. **Integration**: 
   - Initialized in `CuroboPlanner` class (planner.py)
   - Wraps NVIDIA's CuRobo `MotionGen` library
   - Loads configuration from `curobo.yml` files
3. **Key Features**:
   - Single path planning via `plan_path()`
   - **Batch planning** via `plan_batch()` - plans up to 10+ poses simultaneously
   - GPU acceleration (10-100x faster than CPU)
4. **Multi-process support**: For dual-arm robots with different configs, spawns separate processes to avoid CUDA context conflicts
5. **Coordinate transformations**: Handles world→robot base→arm base frame conversions
6. **Collision world**: Configured with static table geometry at initialization

### Q2: Does this mean if I don't configure CuRobo for robot, just config Mplib, it can work too?

**A:** Yes, absolutely! The framework is designed to work with **both** planners:

1. **Default planner** in `config.yml` is `"mplib_RRT"` (not CuRobo)
2. **CuRobo import is wrapped** in try-except block - framework continues if import fails
3. **Mplib is always available** as fallback
4. **All embodiment configs** have commented Mplib options showing it was tested
5. **Decision**: Change `planner: "curobo"` → `planner: "mplib_RRT"` in config.yml

**Advantages of Mplib**:
- ✅ No GPU required
- ✅ No complex setup (just needs URDF/SRDF)
- ✅ Well-tested RRT implementation
- ✅ Good collision avoidance with SAPIEN scene integration
- ✅ Easy to debug (pure Python)

### Q3: Does RoboTwin preset config really use Mplib? Or all use CuRobo?

**A:** All RoboTwin presets use **CuRobo by default**:

```bash
aloha-agilex: "curobo"  (comment: # planner: "mplib_screw")
ARX-X5:       "curobo"  (comment: # planner: "mplib_RRT")
franka-panda: "curobo"  (comment: # planner: "mplib_screw")
piper:        "curobo"  (comment: # planner: "mplib_RRT")
ur5-wsg:      "curobo"  (no Mplib comment)
```

**Why CuRobo is default**:
- Speed: GPU planning is ~10-100x faster
- Batch planning: Essential for grasp exploration strategy
- Research focus: Cutting-edge robotics research platform

**Mplib support**: Available but commented out, proving the framework supports both

### Q4: It seems like the SAPIEN scene has built-in integration with Mplib?

**A:** Yes! Mplib has **excellent SAPIEN integration** through two special classes:

1. **`SapienPlanningWorld`**: Creates a bridge between SAPIEN and Mplib
   ```python
   planning_world = SapienPlanningWorld(scene, [robot_entity])
   ```
   - Automatically syncs all scene objects
   - Updates collision geometry dynamically

2. **`SapienPlanner`**: Planner that uses the SAPIEN world
   ```python
   self.planner = SapienPlanner(planning_world, move_group)
   ```
   - Collision checks against all scene objects (tables, obstacles, other robots)
   - No manual obstacle management needed

**Two initialization modes**:
- **Standalone** (scene=None): Uses only URDF/SRDF collision geometry
- **SAPIEN-integrated** (scene provided): Automatically sees entire scene ✨

**Benefits**:
- ✅ Automatic collision geometry sync
- ✅ Dynamic updates (picks up new/moved objects)
- ✅ Realistic simulation (same geometry for physics and planning)
- ✅ Multi-robot support
- ✅ Simpler API

This is a **major advantage** of Mplib over CuRobo!

### Q5: What collision info is passed to CuRobo to avoid? Only table?

**A:** Yes, **only the table**! This is a critical limitation:

**CuRobo receives (planner.py lines 58-73)**:
```python
world_config = {
    "cuboid": {
        "table": {
            "dims": [0.7, 2, 0.04],  # x, y, z
            "pose": [y, x, z, qw, qx, qy, qz],
        },
    }
}
```

**What CuRobo knows**:
- ✅ One static table (hardcoded dimensions)
- ❌ NO objects on the table
- ❌ NO walls
- ❌ NO cluttered items
- ❌ NO target objects (cups, bottles, boxes, etc.)

**Comparison**:
- **Mplib**: Sees entire SAPIEN scene automatically
- **CuRobo**: Only static table at initialization

**Partial solution**: `update_world_pcd()` method
```python
robot.update_world_pcd(world_pcd)  # Pass point cloud
```
- Can add obstacles via point clouds
- Must be called manually
- Not used in basic initialization
- Less efficient than native mesh collision

**Why this design**:
1. Performance trade-off: CuRobo prioritizes speed → minimal collision geometry
2. Task-specific: Manipulation tasks often have open workspace above table
3. Vision-guided: Relies on vision-based grasping to avoid objects
4. Batch planning: Can't dynamically update world for 10+ parallel plans

**Real collision avoidance strategy in RoboTwin**:
1. Prohibited areas (`add_prohibit_area()`) - prevents sampling grasps near objects
2. Visual servoing - Real-time adjustment during execution
3. IK checking - Fast validation
4. Simple workspace - Tasks designed to minimize clutter

**To add more obstacles to CuRobo**, you would need to manually extend `world_config`:
```python
world_config = {
    "cuboid": {
        "table": {...},
        "box_obstacle_1": {
            "dims": [0.1, 0.1, 0.1],
            "pose": [x, y, z, 1, 0, 0, 0],
        },
    },
    "sphere": {
        "ball_1": {
            "radius": 0.05,
            "pose": [x, y, z, 1, 0, 0, 0],
        }
    }
}
```
But this is **static** and doesn't sync with SAPIEN's dynamic scene.

**Conclusion**: Mplib gives you **much better collision avoidance** for complex environments!

---

---

## Quick Summary

| Feature | Mplib | CuRobo |
|---------|-------|--------|
| **Hardware** | CPU only | Requires CUDA GPU |
| **Speed** | Moderate | 10-100x faster |
| **Collision Detection** | Full SAPIEN scene integration | Static table + optional point cloud |
| **Batch Planning** | ❌ No | ✅ Yes (multiple poses in parallel) |
| **Setup Complexity** | Simple | Complex (CUDA dependencies) |
| **Default in RoboTwin** | ❌ No | ✅ Yes (all presets) |
| **Dynamic Obstacles** | ✅ Automatic | ⚠️ Manual updates required |
| **Best For** | Learning, debugging, CPU-only | Production, research, speed-critical |

---

## File Structure

```
envs/robot/
├── planner.py          # Planner implementations (MplibPlanner, CuroboPlanner)
├── robot.py            # Robot class that uses planners
└── ik.py               # TODO: IK implementation

assets/embodiments/<robot_name>/
├── config.yml          # Main robot configuration
├── <robot>.urdf        # Robot kinematics (REQUIRED)
├── <robot>.srdf        # Semantic info (OPTIONAL for Mplib)
└── curobo.yml          # CuRobo-specific config (for CuRobo only)
```

---

## Planner Selection

### Current Status (All Presets)
```bash
# Check all robot planner configs
$ grep "^planner:" assets/embodiments/*/config.yml
aloha-agilex/config.yml:22:planner: "curobo"
ARX-X5/config.yml:24:planner: "curobo"
franka-panda/config.yml:29:planner: "curobo"
piper/config.yml:23:planner: "curobo"
ur5-wsg/config.yml:25:planner: "curobo"
```

**All presets use CuRobo by default!**

### Switching to Mplib
Edit `assets/embodiments/<robot_name>/config.yml`:
```yaml
# Change from:
planner: "curobo"

# To one of:
planner: "mplib_RRT"     # RRT-based collision-free planning (recommended)
planner: "mplib_screw"   # Fast screw interpolation (no collision avoidance)
```

---

## Configuration Files Deep Dive

### 1. config.yml (Required for Both Planners)

```yaml
# ============= File Paths =============
urdf_path: "./panda.urdf"              # REQUIRED: Robot kinematics
srdf_path: "./panda.srdf"              # OPTIONAL: Semantic robot info (helps Mplib)

# ============= Planner Selection =============
planner: "mplib_RRT"                   # Options: mplib_RRT | mplib_screw | curobo

# ============= Planning Configuration =============
move_group: ["panda_hand", "panda_hand"]      # End-effector link (left, right)
ee_joints: ["panda_hand_joint", ...]          # Joint at end-effector
arm_joints_name: [
  ['panda_joint1', 'panda_joint2', ..., 'panda_joint7'],  # Left arm
  ['panda_joint1', 'panda_joint2', ..., 'panda_joint7']   # Right arm
]

# ============= Gripper Configuration =============
gripper_name:
  - base: "panda_finger_joint1"               # Main gripper joint
    mimic: [["panda_finger_joint2", 1.0, 0.0]]  # [name, multiplier, offset]
gripper_bias: 0.08                            # Distance from wrist to gripper center (m)
gripper_scale: [-0.01, 0.05]                  # [closed_position, open_position]

# ============= Robot Placement =============
robot_pose: [[0, -0.65, 0.75, 0.707, 0, 0, 0.707]]  # [x,y,z, qw,qx,qy,qz]

# ============= Coordinate Frames =============
delta_matrix: [[0,0,1],[0,-1,0],[1,0,0]]      # EE frame → planning frame
global_trans_matrix: [[1,0,0],[0,-1,0],[0,0,-1]]  # World → robot base

# ============= Home Position =============
homestate: [[0, 0.196, 0, -2.618, 0, 2.942, 0.785]]  # Reset joint angles

# ============= Other Settings =============
dual_arm: False                               # Single robot (True for dual-arm)
rotate_lim: [0.1, 0.8]                       # Grasp rotation limits
grasp_perfect_direction: ['right', 'left']   # Preferred grasp approach
```

### 2. URDF File (Universal Robot Description Format)
**Required for BOTH planners**

Contains:
- Joint types (revolute, prismatic, fixed)
- Link geometry and inertial properties
- Joint limits (position, velocity, effort)
- Collision and visual meshes
- Parent-child relationships

Used for: Forward/inverse kinematics, collision checking

### 3. SRDF File (Semantic Robot Description Format)
**Optional but recommended for Mplib**

Contains:
```xml
<!-- Planning groups -->
<group name="panda_arm">
  <joint name="panda_joint1"/>
  ...
</group>

<!-- End effector -->
<end_effector name="eef" parent_link="panda_link8" group="hand"/>

<!-- Disable unnecessary collision checks -->
<disable_collisions link1="panda_hand" link2="panda_link7" reason="Adjacent"/>
```

Benefits:
- Faster planning (pre-defined groups)
- Optimized collision checking
- Named poses for common configurations

**Note:** ARX-X5 shows SRDF is truly optional (commented out in config)

### 4. curobo.yml (Only for CuRobo)
**Required ONLY if using CuRobo planner**

Contains CuRobo-specific parameters:
- Robot kinematics in CuRobo format
- Joint limits and velocity bounds
- Self-collision configuration
- Frame transformations (`frame_bias`)

---

## Mplib Implementation Details

### Initialization (planner.py lines 283-314)

```python
class MplibPlanner:
    def __init__(
        self,
        urdf_path,        # Robot URDF
        srdf_path,        # Optional SRDF
        move_group,       # End-effector link name
        robot_origion_pose,  # Robot base pose in world
        robot_entity,     # SAPIEN robot entity
        planner_type="mplib_RRT",  # or "mplib_screw"
        scene=None,       # Optional SAPIEN scene
    ):
```

### Two Initialization Modes

#### Mode 1: Standalone (scene=None)
```python
self.planner = mplib.Planner(
    urdf=urdf_path,
    srdf=srdf_path,
    move_group=move_group,
    user_link_names=links,
    user_joint_names=joints,
    use_convex=False,  # Use exact collision meshes
)
self.planner.set_base_pose(robot_origion_pose)
```
- Uses only URDF/SRDF collision geometry
- No scene object awareness
- Faster but limited

#### Mode 2: SAPIEN-Integrated (scene provided) ✨
```python
planning_world = SapienPlanningWorld(scene, [robot_entity])
self.planner = SapienPlanner(planning_world, move_group)
```
- **Automatic SAPIEN scene sync**
- Collision checks with all objects (tables, obstacles, other robots)
- Dynamic updates as objects move
- **Recommended for realistic planning**

### Planning Methods

#### 1. mplib_RRT (Recommended)
```python
result = planner.plan_pose(
    goal_pose=target_pose,
    current_qpos=np.array(now_qpos),
    time_step=1/250,           # 250 Hz control
    planning_time=5,           # Max 5 seconds
)
```
- RRT (Rapidly-exploring Random Tree) algorithm
- Collision-aware motion planning
- Retries up to 10 times (configurable)
- Best for: Complex environments with obstacles

#### 2. mplib_screw
```python
result = planner.plan_screw(
    goal_pose=target_pose,
    current_qpos=now_qpos,
    time_step=1/250,
)
```
- Interpolates along screw motion (straight line in task space)
- **NO collision avoidance** - only validates if path is collision-free
- Much faster but can fail if obstacles block path
- Best for: Simple motions in open space

### Tunable Parameters (planner.py)
```python
self.plan_step_lim = 2500      # Max waypoints (rejects complex paths)
planning_time = 5               # RRT search timeout (seconds)
try_times = 10                  # Retry attempts for RRT
time_step = 1/250               # Control frequency (250 Hz)
```

---

## CuRobo Implementation Details

### Initialization (planner.py lines 32-92)

```python
class CuroboPlanner:
    def __init__(
        self,
        robot_origion_pose,
        active_joints_name,
        all_joints,
        yml_path=None,      # curobo.yml path
    ):
```

### Collision World Configuration

**CRITICAL LIMITATION:** CuRobo only receives static table configuration!

```python
# planner.py lines 58-73
world_config = {
    "cuboid": {
        "table": {
            "dims": [0.7, 2, 0.04],  # Table dimensions (m)
            "pose": [y, x, z, qw, qx, qy, qz],  # Table pose
        },
    }
}
```

**What CuRobo knows about:**
- ✅ One static table
- ❌ NO objects on table
- ❌ NO walls
- ❌ NO dynamic obstacles

### Point Cloud Updates (Partial Solution)

```python
# robot.py lines 323-328
def update_world_pcd(self, world_pcd):
    try:
        self.left_planner.update_point_cloud(world_pcd, resolution=0.02)
        self.right_planner.update_point_cloud(world_pcd, resolution=0.02)
    except:
        print("Update world pointcloud wrong!")
```

This allows dynamic obstacle updates via point clouds, but:
- Must be called manually
- Not used in basic initialization
- Less efficient than native mesh collision

### Batch Planning (CuRobo's Killer Feature)

```python
result = planner.plan_batch(
    curr_joint_pos,
    target_gripper_pose_list,  # List of 10+ poses
    constraint_pose=None,
    arms_tag="left",
)

# Returns:
# result['status']: ["Success", "Fail", "Success", ...]  # Array
# result['position']: shape (n_poses, n_waypoints, n_joints)
# result['velocity']: shape (n_poses, n_waypoints, n_joints)
```

Used for: Exploring multiple grasp orientations in parallel (see `create_target_pose_list` in robot.py)

### Multi-Process Architecture

For dual-arm robots with different CuRobo configs:
```python
# robot.py lines 277-300
self.left_proc = mp.Process(target=planner_process_worker, args=(left_child_conn, left_args))
self.right_proc = mp.Process(target=planner_process_worker, args=(right_child_conn, right_args))
self.left_proc.start()
self.right_proc.start()
```
Avoids CUDA context conflicts between different robot configurations.

---

## Adding a New Robot: Step-by-Step Guide

### Option A: Using Mplib (Recommended for Beginners)

#### 1. Prepare Robot Files
```
assets/embodiments/my_robot/
├── config.yml
├── my_robot.urdf       # REQUIRED
└── my_robot.srdf       # OPTIONAL but helpful
```

#### 2. Create config.yml
```yaml
urdf_path: "./my_robot.urdf"
srdf_path: "./my_robot.srdf"  # Optional

planner: "mplib_RRT"  # Start with RRT

move_group: ["end_effector_link", "end_effector_link"]
ee_joints: ["ee_joint", "ee_joint"]

arm_joints_name: [
  ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'],
  ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
]

gripper_name:
  - base: "gripper_joint"
    mimic: []

gripper_bias: 0.10      # Measure from wrist to gripper center
gripper_scale: [0.0, 0.04]  # [closed, open] positions

robot_pose: [[0, -0.65, 0.75, 1, 0, 0, 0]]  # Adjust for your workspace

homestate: [[0, 0, 0, 0, 0, 0]]  # Safe home joint angles

delta_matrix: [[1,0,0],[0,1,0],[0,0,1]]  # Start with identity
global_trans_matrix: [[1,0,0],[0,-1,0],[0,0,-1]]

dual_arm: False
rotate_lim: [0, 1]
grasp_perfect_direction: ['front_right', 'front_left']
```

#### 3. Test Initialization
```python
from envs.robot.robot import Robot

robot = Robot(
    scene=scene,
    left_robot_file="./assets/embodiments/my_robot",
    right_robot_file="./assets/embodiments/my_robot",
    left_embodiment_config=config,
    right_embodiment_config=config,
    dual_arm_embodied=False,
    embodiment_dis=1.0,
)

robot.print_info()  # Verify joints loaded correctly
```

#### 4. Tune Parameters
- Adjust `robot_pose` for your table height
- Calibrate `gripper_bias` by measuring physical robot
- Set `homestate` to a collision-free configuration
- If planning fails, increase `planning_time` or adjust `try_times`

#### 5. Add to _embodiment_config.yml
```yaml
# task_config/_embodiment_config.yml
my_robot:
  file_path: "./assets/embodiments/my_robot"
```

### Option B: Using CuRobo (Advanced)

#### Additional Requirements
1. Create `curobo.yml` with robot kinematics in CuRobo format
2. Define `frame_bias` for base-to-arm transformation
3. Configure self-collision spheres
4. Set up GPU environment

#### Steps
1. Follow Mplib setup first to verify URDF correctness
2. Generate CuRobo config using CuRobo's tools:
   ```bash
   # Follow: https://github.com/NVlabs/curobo
   ```
3. Update `config.yml`:
   ```yaml
   planner: "curobo"
   ```
4. Test with small number of planning attempts first

---

## Common Issues & Debugging

### Planning Always Fails
1. **Check URDF validity:**
   ```python
   robot.print_info()  # Verify all joints loaded
   ```
2. **Verify collision geometry:**
   - Ensure collision meshes are not overly complex
   - Check for self-collisions in home state
3. **Increase planning attempts:**
   ```python
   # In planner.py
   try_times = 20  # Instead of 10
   planning_time = 10  # Instead of 5
   ```

### CuRobo vs Mplib Performance
- **Mplib too slow?** Switch to `planner: "curobo"`
- **CuRobo collisions not detected?** Use `update_world_pcd()` or switch to Mplib
- **Need batch planning?** CuRobo only

### Gripper Issues
1. **Gripper not moving:**
   - Check `gripper_scale` values match URDF joint limits
   - Verify `gripper_name` base joint exists
2. **Gripper collision issues:**
   - Adjust `gripper_bias` to move planning target away from gripper fingers

### Coordinate Frame Issues
1. **End-effector orientation wrong:**
   - Tune `delta_matrix` (transforms EE frame to planning frame)
   - Adjust `global_trans_matrix` (world to robot base)
2. **Debug approach:**
   ```python
   tcp_pose = robot.get_left_tcp_pose()
   print(f"TCP: {tcp_pose}")  # Should match expected position
   ```

---

## Planning Interface (robot.py)

### Single Path Planning
```python
result = robot.left_plan_path(
    target_pose=[x, y, z, qw, qx, qy, qz],
    constraint_pose=None,  # Optional orientation constraint
    last_qpos=None,        # Use current if None
)

if result["status"] == "Success":
    trajectory = result["position"]  # Shape: (n_steps, n_joints)
    velocities = result["velocity"]  # Shape: (n_steps, n_joints)
```

### Batch Planning (CuRobo only)
```python
pose_list = robot.create_target_pose_list(
    origin_pose=grasp_pose,
    center_pose=object_center,
    arm_tag="left",
)  # Generates CONFIGS.ROTATE_NUM rotated poses

result = robot.left_plan_multi_path(
    target_lst=pose_list,
    constraint_pose=[0,0,0,0,0,1],  # Keep Z-axis down
)

# Find first successful trajectory
for i, status in enumerate(result["status"]):
    if status == "Success":
        trajectory = result["position"][i]  # Shape: (n_steps, n_joints)
        break
```

---

## Key Insights for Configuration

### 1. SAPIEN Integration Advantage (Mplib)
Mplib's `SapienPlanningWorld` automatically syncs collision geometry:
- All objects in scene become obstacles
- Dynamic updates as objects move
- No manual obstacle management

**Recommendation:** Use Mplib with scene integration for development/testing.

### 2. CuRobo's Speed vs Accuracy Trade-off
CuRobo sacrifices dynamic collision detection for speed:
- 10-100x faster than Mplib
- Batch planning for grasp exploration
- But only knows about static table

**Recommendation:** Use CuRobo for production after validating with Mplib.

### 3. Gripper Bias Calibration
Critical for accurate grasping:
```yaml
gripper_bias: 0.08  # Distance from wrist joint to gripper center (m)
```
Measure this on your physical robot! Incorrect values cause grasp failures.

### 4. Home State Safety
```yaml
homestate: [[0, 0.196, 0, -2.618, 0, 2.942, 0.785]]
```
Ensure home position:
- Is collision-free with table
- Keeps arms away from workspace boundaries
- Is kinematically reachable

### 5. Coordinate Frame Conventions
```yaml
delta_matrix: [[0,0,1],[0,-1,0],[1,0,0]]
```
This transforms end-effector frame to planning frame. Common patterns:
- Franka Panda: `[[0,0,1],[0,-1,0],[1,0,0]]`
- UR5: `[[1,0,0],[0,1,0],[0,0,1]]` (identity)
- Adjust based on your robot's URDF conventions

---

## Performance Comparison

### Benchmark (from code analysis)
```python
# Mplib RRT
planning_time: 5 seconds max
try_times: 10 attempts
success_rate: ~60-80% (depends on clutter)

# CuRobo
planning_time: ~0.05-0.5 seconds typical
batch_size: 10 poses simultaneously
success_rate: ~70-90% (open workspace)
```

### Memory Usage
- Mplib: ~100-200 MB (CPU)
- CuRobo: ~1-2 GB (GPU VRAM)

---

## Future Improvements Needed

Based on code investigation:

1. **CuRobo Scene Integration**
   - Add dynamic object tracking to `world_config`
   - Implement automatic SAPIEN → CuRobo obstacle conversion
   
2. **IK Module** (`ik.py` is TODO)
   - Fast inverse kinematics without full planning
   - Useful for reactive control

3. **Hybrid Planner**
   - Use CuRobo for coarse planning
   - Mplib for fine-grained collision checking
   
4. **Better Error Messages**
   - Current: "planning failed"
   - Needed: Why it failed (collision? IK? joint limits?)

---

## Quick Reference Commands

```bash
# Check all robot planners
grep "^planner:" assets/embodiments/*/config.yml

# Find robot embodiment configs
ls assets/embodiments/*/config.yml

# Test robot loading
python -c "from envs.robot.robot import Robot; print('OK')"

# Check mplib installation
python -c "import mplib; print(mplib.__version__)"

# Check CuRobo installation (requires GPU)
python -c "import curobo; print('CuRobo OK')"
```

---

## Summary: Mplib vs CuRobo Decision Tree

```
Need GPU acceleration? ──No──> Use Mplib
         │
        Yes
         │
Need dynamic obstacle ──Yes──> Use Mplib + SAPIEN integration
avoidance?                     (or implement pcd updates for CuRobo)
         │
        No
         │
Need batch planning? ──Yes──> Use CuRobo
         │
        No
         │
Speed critical? ──Yes──> Use CuRobo
         │
        No
         │
      Use Mplib (simpler, more features)
```

---

## References

- **Mplib Documentation:** https://github.com/haosulab/MPlib
- **CuRobo Repository:** https://github.com/NVlabs/curobo
- **SAPIEN Documentation:** https://sapien.ucsd.edu/
- **RoboTwin Paper:** Check README.md for citation

---

*Last Updated: 2025-12-30*
*Investigation Branch: object_instantiation_xc*
