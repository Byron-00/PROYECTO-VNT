# Reactive Follow the Gap — Technical Tutorial

## 1. Introduction

The Sakhir autonomous racing controller is implemented in Python using ROS 2 Humble.

The system uses LiDAR data to choose a driving direction and publishes Ackermann steering and speed commands.

The implementation consists of two ROS 2 nodes:

- `sakhir_gap_controller`: autonomous vehicle control.
- `sakhir_lap_timer`: automatic lap counting and timing.

## 2. Main Controller

Source file:

`src/sakhir_gap_controller/sakhir_gap_controller/gap_controller.py`

The controller is based on Reactive Follow the Gap.

### Step 1: LiDAR Acquisition

The controller subscribes to `/scan` using the LaserScan message type.

Each scan contains obstacle distances and their corresponding angles.

### Step 2: Preprocessing

Invalid and out-of-range readings are handled before navigation calculations.

The scan is converted into obstacle distances and Cartesian coordinates.

### Step 3: Disparity Extension

Large differences between adjacent LiDAR readings indicate possible obstacle boundaries.

Nearby obstacles are inflated to account for the vehicle's physical size and improve clearance.

### Step 4: Gap Selection

The controller identifies navigable free-space regions and selects a target heading.

Target continuity and centrality help reduce oscillation between different directions.

### Step 5: Steering Calculation

The selected heading is converted into an Ackermann steering command using the vehicle's wheelbase.

Steering filtering and steering-rate limitations help reduce rapid corrections.

### Step 6: Trajectory Evaluation

Candidate steering commands are evaluated using an approximate Ackermann vehicle trajectory.

The planned path is checked against LiDAR obstacles using the estimated vehicle footprint.

### Step 7: Adaptive Speed Control

The commanded speed depends on several constraints:

- Maximum configured speed.
- Requested steering and path curvature.
- Available obstacle clearance.
- Braking distance.
- Acceleration limits.

The final speed command is published through `/drive`.

## 3. ROS 2 Communication

| Node | Subscribes to | Publishes to |
|---|---|---|
| `sakhir_gap_controller` | `/scan`, `/ego_racecar/odom` | `/drive` |
| `sakhir_lap_timer` | `/ego_racecar/odom` | Console output |

## 4. Lap Counter and Timer

Source file:

`src/sakhir_gap_controller/sakhir_gap_controller/lap_timer.py`

The node receives odometry measurements and establishes a virtual finish line using the initial position and heading.

After leaving the starting area, the system detects a valid crossing in the original travel direction.

Each valid crossing increments the lap count and records the elapsed time.

The results are displayed in the terminal for the racing demonstration.

The timing system uses wall-clock time. Its virtual finish-line detection must be checked against the actual vehicle position during the run.

## 5. Execution

Start the simulator, then start the lap timer, and finally start the autonomous controller.

All new terminals must source ROS 2 Humble and the relevant workspace.

Detailed commands are provided in the root README.

## 6. Testing Method

The controller was evaluated on the Sakhir circuit.

The development included:

1. Initial Follow the Gap implementation.
2. Steering oscillation reduction.
3. Obstacle-clearance improvements.
4. Adaptive speed tuning.
5. Cornering optimization.
6. Ten-lap validation.

## 7. Limitations

LiDAR-based reactive navigation depends on sensor range, obstacle visibility, and the chosen steering strategy.

Collision checking uses an approximate geometric vehicle model.

The maximum speed parameter is not the same as sustained speed or average lap speed.

The lap counter uses a virtual gate and requires a consistent starting position.

## 8. Demonstration

The repository includes screenshots of the vehicle in RViz and a ten-lap demonstration video with the lap counter and timing output.

## 9. Conclusion

The first project stage demonstrates autonomous racing using Reactive Follow the Gap, ROS 2 Humble, and F1TENTH Gym.

The controller and timer source files are provided for reproduction, inspection, and future development.
