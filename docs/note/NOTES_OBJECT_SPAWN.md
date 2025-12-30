# Object Spawn Investigation - RoboTwin Framework

## Overview
This document summarizes how objects (tables, bottles, bowls, etc.) are instantiated in the RoboTwin framework, based on codebase investigation.

---

## Investigation Questions & Answers

### Q1: How does this framework instantiate objects such as table, bottle, bowl?

**Answer:**

The framework uses **two different approaches** depending on the object type:

1. **Tables** - Procedural creation using primitive shapes:
   - Uses `create_table()` function which builds a table from box primitives (tabletop + 4 legs)
   - Called from `Base_Task.create_table_and_wall()` during scene setup
   - Returns a `sapien.Entity`

2. **3D Objects (bottles, bowls, etc.)** - Mesh-based loading:
   - Uses `create_actor()` function which loads 3D mesh files (GLB/OBJ format)
   - Loads mesh files from `assets/objects/{modelname}/` directory
   - Supports separate collision and visual meshes for optimization
   - Loads metadata from `model_data{model_id}.json` (scale, extents, contact points, etc.)
   - Uses Sapien's ActorBuilder to create physics-enabled actors
   - Returns an `Actor` wrapper class that provides convenient methods

**Example usage:**
```python
# Bottle creation
bottle = create_actor(
    self,
    bottle_pose,
    modelname="114_bottle",
    convex=True,
    model_id=model_id,
)

# Bowl creation
bowl = create_actor(
    self, 
    pose=bowl_pose, 
    modelname="002_bowl", 
    model_id=3, 
    convex=True
)
```

---

### Q2: What's the bounding box information used for in RoboTwin?

**Answer:**

Bounding box information (`extents`) is primarily used for **spatial collision avoidance** during scene setup and object placement:

1. **Prohibited Area Calculation** (`add_prohibit_area()`):
   - Computes the 2D X-Y projection of an object's 3D bounding box
   - Transforms the 8 corner points to world coordinates using the object's pose
   - Calculates min/max X and Y values with optional padding
   - Stores as `[x_min, y_min, x_max, y_max]` in the `prohibited_area` list
   - Prevents overlapping object placements during scene generation

2. **Cluttered Scene Generation** (`rand_create_cluttered_actor()`):
   - Uses extents to calculate:
     - `z_max`: Maximum height for vertical stacking (`(extents[1] + center[1]) * scale[1]`)
     - `radius`: Horizontal collision radius for overlap checking (`max(extents[0] * scale[0], extents[2] * scale[2]) / 2`)
   - Ensures objects don't overlap when randomly placed in cluttered scenes

3. **Source of Extents**:
   - Primary source: `model_data{model_id}.json` → `extents` field
   - Alternative: `bounding_box.json` (for URDF objects) → converted to extents via `(max - min)`
   - Fallback: Default `[0.1, 0.1, 0.1]` if not found in metadata

**Key insight:** The bounding box extents are critical for preventing collisions during scene initialization, but are **not directly used** by the robot path planner for runtime collision avoidance (that's handled by the physics engine).

---

## Object Types & Instantiation Methods

### 1. Tables - Procedural Creation
- **Function**: `create_table()` in `envs/utils/create_actor.py` (lines 324-389)
- **Method**: Built from primitive shapes (box primitives for tabletop + 4 legs)
- **Returns**: `sapien.Entity`
- **Usage**: Called from `Base_Task.create_table_and_wall()`
- **Properties**: Static by default, supports textures, customizable dimensions

### 2. 3D Objects (Bottles, Bowls, etc.) - Mesh-based Creation
- **Primary Function**: `create_actor()` in `envs/utils/create_actor.py` (lines 501-559)
- **Method**: Loads 3D mesh files (GLB/OBJ format) from assets directory
- **Returns**: `Actor` wrapper class
- **Key Parameters**:
  - `modelname`: Object identifier (e.g., "114_bottle", "002_bowl")
  - `model_id`: Optional variant ID (for objects with multiple variants)
  - `convex`: Boolean for collision type (convex vs nonconvex)
  - `is_static`: Boolean for physics behavior

---

## Object Instantiation Pipeline

### For 3D Objects (`create_actor`):

1. **File Resolution**
   ```
   assets/objects/{modelname}/
     ├── collision/          (optional - checked first)
     │   └── base{model_id}.glb or textured{model_id}.obj
     ├── visual/             (optional - checked first)
     │   └── base{model_id}.glb or textured{model_id}.obj
     ├── base{model_id}.glb  (fallback if subdirs don't exist)
     ├── textured{model_id}.obj
     └── model_data{model_id}.json  (metadata)
   ```

2. **Metadata Loading**
   - Loads `model_data{model_id}.json` containing:
     - `scale`: Object scaling factors
     - `extents`: Bounding box dimensions (used for collision avoidance)
     - `center`: Object center point
     - Contact points, target points, functional points, etc.

3. **Actor Building**
   - Creates `ActorBuilder` from scene
   - Sets physics body type (static/dynamic)
   - Adds collision shape from collision file (convex or nonconvex)
   - Adds visual geometry from visual file
   - Applies scale from metadata
   - Sets initial pose
   - Builds and returns `Actor` wrapper

4. **Actor Wrapper**
   - `Actor` class in `envs/utils/actor_utils.py`
   - Wraps underlying Sapien entity
   - Provides access to contact points, target points, functional points
   - Stores metadata from JSON config

---

## Bounding Box Usage

### Purpose
Bounding box information (`extents`) is primarily used for **spatial collision avoidance** during scene setup.

### Key Uses:

1. **Prohibited Area Calculation** (`add_prohibit_area()` in `_base_task.py`)
   - Computes 2D X-Y projection of object's bounding box
   - Adds padding for safety margin
   - Stores as `[x_min, y_min, x_max, y_max]` in `prohibited_area` list
   - Used to prevent object placement overlaps

2. **Cluttered Scene Generation** (`rand_create_cluttered_actor()`)
   - Uses extents to calculate:
     - `z_max`: Maximum height for vertical stacking
     - `radius`: Horizontal collision radius (from X and Z extents)
   - Prevents overlapping placements during random scene generation

3. **Source of Extents**:
   - Primary: `model_data{model_id}.json` → `extents` field
   - Alternative: `bounding_box.json` (for URDF objects) → converted to extents
   - Fallback: `[0.1, 0.1, 0.1]` default if not found

---

## Key Functions Reference

### Object Creation Functions

| Function | Location | Purpose | Returns |
|----------|----------|---------|---------|
| `create_actor()` | `envs/utils/create_actor.py:501` | Main function for 3D mesh objects | `Actor` |
| `create_table()` | `envs/utils/create_actor.py:324` | Procedural table creation | `sapien.Entity` |
| `create_obj()` | `envs/utils/create_actor.py:393` | Alternative for OBJ files | `Actor` |
| `create_glb()` | `envs/utils/create_actor.py:440` | Alternative for GLB files | `Actor` |
| `create_urdf_obj()` | `envs/utils/create_actor.py:563` | For articulated URDF objects | `ArticulationActor` |
| `rand_create_actor()` | `envs/utils/rand_create_actor.py:183` | Creates actor at random pose | `Actor` |

### Spatial Management Functions

| Function | Location | Purpose |
|----------|----------|---------|
| `add_prohibit_area()` | `envs/_base_task.py:649` | Adds object's bounding box to prohibited areas |
| `rand_create_cluttered_actor()` | `envs/utils/rand_create_cluttered_actor.py:189` | Creates objects in cluttered scenes with overlap checking |

---

## Typical Usage Pattern in Tasks

```python
class example_task(Base_Task):
    def load_actors(self):
        # 1. Create object with random pose
        obj_pose = rand_pose(xlim=[-0.3, 0.3], ylim=[-0.15, 0.15], ...)
        
        # 2. Instantiate object
        self.object = create_actor(
            scene=self,
            pose=obj_pose,
            modelname="114_bottle",
            model_id=1,
            convex=True,
        )
        
        # 3. Set physics properties
        self.object.set_mass(0.01)
        
        # 4. Add prohibited area for collision avoidance
        self.add_prohibit_area(self.object, padding=0.1)
        
        # 5. Optionally add manual prohibited areas
        self.prohibited_area.append([x_min, y_min, x_max, y_max])
```

---

## File Structure

```
assets/objects/
  ├── {modelname}/              # e.g., "114_bottle", "002_bowl"
  │   ├── base{id}.glb         # Visual/collision mesh
  │   ├── textured{id}.obj     # Alternative format
  │   ├── collision/            # Optional separate collision meshes
  │   ├── visual/               # Optional separate visual meshes
  │   ├── model_data{id}.json  # Metadata (extents, scale, contact points, etc.)
  │   └── bounding_box.json     # Optional (for URDF objects)
  └── ...
```

---

## Important Implementation Details

1. **Pose Preprocessing**: `preprocess()` function adjusts Z-coordinate based on `table_z_bias` if scene is not a direct `sapien.Scene`

2. **Scale Application**: Scale is applied from `model_data.json`, not from function parameter (parameter is ignored if JSON exists)

3. **Collision vs Visual**: Framework supports separate collision and visual meshes for performance optimization

4. **Model ID Variants**: Objects can have multiple variants (e.g., `base0.glb`, `base1.glb`) selected via `model_id` parameter

5. **Default Extents**: If extents not found, defaults to `[0.1, 0.1, 0.1]` in prohibited area calculations

---

## Notes for Future Development

- **Adding New Objects**: Ensure proper directory structure in `assets/objects/{modelname}/` with mesh files and `model_data.json`
- **Modifying Spawn Logic**: Main entry point is `create_actor()`, but consider variants (`create_obj()`, `create_glb()`) for specific needs
- **Collision Avoidance**: Bounding boxes are critical for scene generation - ensure accurate extents in metadata
- **Performance**: Consider using convex collision for simple objects, nonconvex for complex geometry
- **Prohibited Areas**: Used for spatial planning but may need integration with actual path planner collision checking

---

## Related Files

- `envs/utils/create_actor.py` - Core object creation functions
- `envs/utils/actor_utils.py` - Actor wrapper class
- `envs/utils/rand_create_actor.py` - Random placement utilities
- `envs/utils/rand_create_cluttered_actor.py` - Cluttered scene generation
- `envs/_base_task.py` - Base task class with `add_prohibit_area()`
- `script/create_object_data.py` - Tool for generating object metadata

