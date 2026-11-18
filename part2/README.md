# F1TENTH Autonomous Racing — Part 2

## Static Obstacle Avoidance and Autonomous Overtaking

**Author:** Byron Ortega  
**Project:** PROYECTO-VNT  
**Platform:** Ubuntu 22.04, ROS 2 Humble  
**Simulator:** F1TENTH Gym ROS  
**Circuit:** Sakhir  
**Navigation:** Reactive Follow the Gap  

---

## 1. Project Overview

Part 2 extends the autonomous racing system
developed during Part 1.

The objective is to improve the reactive controller
so that it can avoid static obstacles and perform
autonomous overtaking maneuvers.

The project includes two competition trials:
static obstacle avoidance and dynamic obstacle
avoidance with two autonomous vehicles.

The stable Part 1 controller is preserved as
the starting point for further development.

**[View Part 1 Documentation](../README.md)**

---

## 2. Competition Requirements

### Trial 1 — Static Obstacle Challenge

Five static obstacles must be placed on the
assigned racing circuit.

The autonomous vehicle must successfully
complete **10 consecutive laps without collisions**.

The objective is to evaluate obstacle detection,
reactive avoidance, steering stability, and
speed control.

### Trial 2 — Dynamic Obstacle and Overtaking

A second autonomous vehicle must be introduced
into the simulation.

The second vehicle must start **5 meters ahead**
of the primary vehicle.

The primary vehicle must autonomously overtake
the second vehicle, which acts as a dynamic obstacle.

Both vehicles must use autonomous control
algorithms developed during Part 1.

Both vehicles must complete **10 consecutive
laps without collisions**.

---

## 3. Development Environment

The project uses:

- Ubuntu 22.04
- ROS 2 Humble
- Python 3
- F1TENTH Gym ROS
- LiDAR-based navigation
- Ackermann steering
- RViz visualization

### Environment Installation

The simulation installation tutorial is available at:

[F1TENTH Installation Repository](https://github.com/widegonz/F1Tenth-Repository)

---

## 4. Development Strategy

The project will be developed and validated
through independent stages.

| Stage | Objective | Status |
|---|---|---|
| 1 | Repository and documentation setup | In progress |
| 2 | Five static obstacles on Sakhir | Planned |
| 3 | LiDAR detection and avoidance | Planned |
| 4 | Ten collision-free laps | Planned |
| 5 | Second autonomous vehicle | Planned |
| 6 | Independent vehicle controllers | Planned |
| 7 | Dual-vehicle RViz visualization | Planned |
| 8 | Autonomous overtaking | Planned |
| 9 | Final ten-lap competition | Planned |

Only completed and verified experiments will
be marked as successful.

---

## 5. Static Obstacle Implementation

This section will document:

- Original map preservation.
- Modified Sakhir map generation.
- Obstacle dimensions and positions.
- LiDAR obstacle detection.
- Controller adjustments.
- Simulation screenshots.
- Ten-lap validation results.

**Status:** Not yet tested.

---

## 6. Two-Vehicle Racing System

This section will document:

- Two-agent simulator configuration.
- Vehicle starting positions.
- Five-meter initial separation.
- ROS 2 topic configuration.
- Independent autonomous controllers.
- Dynamic obstacle detection.
- Safe overtaking maneuvers.

**Status:** Not yet tested.

---

## 7. Advanced RViz Visualization

A dual-vehicle visualization system is planned.

| Element | Display |
|---|---|
| Primary vehicle | Green trajectory |
| Opponent vehicle | Orange trajectory |
| Start/finish line | Visual race marker |
| Vehicle speed | Floating telemetry |
| Lap number | Independent lap counters |
| Lap times | Current, last, and best |
| Vehicle separation | Distance indicator |

Each trajectory will reset when its
corresponding vehicle completes a lap.

The visualization will operate independently
from the autonomous driving controllers.

**Status:** Planned.

---

## 8. Experimental Evidence

Screenshots will be stored in:

`../media/part2/`

### Planned Evidence

| File | Description |
|---|---|
| `01-baseline-sakhir.png` | Original Sakhir circuit |
| `02-obstacle-map.png` | Five static obstacles |
| `03-obstacle-lidar.png` | LiDAR obstacle detection |
| `04-five-obstacles-rviz.png` | Obstacle avoidance |
| `05-static-race-results.png` | Ten-lap static trial |
| `06-two-cars-spawn.png` | Two autonomous vehicles |
| `07-dual-trajectory.png` | Green and orange trajectories |
| `08-overtaking.png` | Autonomous overtaking |
| `09-two-cars-results.png` | Final racing results |

Images and video links will be included
after their corresponding tests are completed.

---

## 9. Testing and Results

| Test | Result | Evidence |
|---|---|---|
| Original map baseline | Pending | Pending |
| Five static obstacles | Pending | Pending |
| Ten laps with obstacles | Pending | Pending |
| Two-vehicle simulation | Pending | Pending |
| Autonomous overtaking | Pending | Pending |
| Ten laps with both cars | Pending | Pending |

Actual results and measured times will
be added after testing.

---

## 10. Development Progress

The detailed engineering log is available here:

[Part 2 Development Log](../docs/part2/PROGRESS.md)

---

## 11. References

- [F1TENTH Official Website](https://f1tenth.org/)
- [F1TENTH Gym](https://github.com/f1tenth/f1tenth_gym)
- [F1TENTH Gym ROS](https://github.com/f1tenth/f1tenth_gym_ros)
- [ROS 2 Humble](https://docs.ros.org/en/humble/)

---

**Developed by Byron Ortega — PROYECTO-VNT**
