# UGV System Architecture

## 1. Purpose

This document defines the agreed functional split between Raspberry Pi #1, Raspberry Pi #2, the MCU, sensors, and the drive hardware. ROS 2 node names, topic names, transport choices, and rates are not yet fixed and are marked `TBD`.

## 2. Responsibility split

### 2.1 Raspberry Pi #1: SLAM computer

Connected primary sensor:

- Front Intel RealSense D455.

Responsibilities:

- Front visual/depth sensing.
- Visual SLAM and localization.
- Map and pose generation.
- Future global path or mission-level planning, subject to performance validation.
- Publish localization/SLAM health and the minimum pose/map information required by Pi #2.

Pi #1 determines where the robot is and, in the future, where it should go. Exact SLAM package, map representation, planner placement, and update rates are `TBD`.

### 2.2 Raspberry Pi #2: vehicle-control and safety computer

Connected devices:

- Left and right Intel RealSense D435 cameras through a powered USB 3.0 hub.
- LiDAR through the powered hub or a direct interface; exact connection is `TBD`.
- MCU through USB; exact protocol is `TBD`.
- External operator input such as keyboard, joystick, or another control source.

Responsibilities:

- Receive manual/external commands.
- Receive autonomous motion intent from Pi #1.
- Process side depth data for obstacle and clearance assistance.
- Process LiDAR data for local obstacle/range awareness.
- Arbitrate manual, autonomous, stop, and safety commands.
- Apply local motion constraints or stop commands before sending a final command to the MCU.
- Collect MCU feedback and system health for publication to the rest of the system.

Pi #2 is the single intended source of normal motion commands to the MCU. It decides whether a requested movement is locally safe to execute.

### 2.3 MCU: low-level drive controller

Responsibilities:

- Receive final drive targets from Pi #2.
- Read left and right quadrature encoders.
- Drive the MDD10A PWM/direction inputs.
- Initially support low-speed open-loop testing; later implement encoder-based closed-loop speed control.
- Enforce a communication watchdog and stop the motors when commands time out.
- Report encoder data and low-level controller status to Pi #2.

The MCU does not perform SLAM, route planning, or mission decisions.

### 2.4 MDD10A and motors

- The MDD10A converts the MCU logic-level commands into motor power for the two tracked-drive motors.
- The motor Hall encoders return quadrature signals to the MCU.
- Final PWM frequency, direction polarity, braking/coasting behavior, acceleration limits, and speed-controller gains are `TBD`.

## 3. Data and command flow

```text
Front D455
  -> Pi #1: SLAM / localization / map
       -> pose, odometry or localization status (TBD interface)
          -> Pi #2

Operator input / future navigation command
  -> Pi #2: command arbitration + side/LiDAR safety constraints
       -> final velocity or wheel target command (TBD)
          -> MCU
             -> PWM/DIR
                -> MDD10A
                   -> left/right motors

Motor encoders
  -> MCU
     -> wheel feedback / controller status
        -> Pi #2
           -> odometry and system status consumers (TBD)

Left/right D435 + LiDAR
  -> Pi #2
     -> clearance / obstacle assessment
        -> command limiting or stop
```

## 4. Control layers

| Layer | Name | Main owner | Scope |
|---|---|---|---|
| L4 | Operator / external control | Operator UI or remote system | Mission start/stop, goal selection, manual command, monitoring, emergency-stop request |
| L3 | Autonomy and mission | Primarily Pi #1; final placement TBD | SLAM, localization, map, global intent, future path/mission planning |
| L2 | Vehicle control and sensor integration | Pi #2 | Input arbitration, side/LiDAR processing, local safety constraints, final command generation |
| L1 | Real-time drive control | MCU | Encoder acquisition, wheel control, PWM/DIR output, command watchdog |
| L0 | Physical I/O | Sensors, MDD10A, motors, power hardware | Measurement and actuation |

Lower safety layers may reject or override a higher-layer motion request. A higher-level command never bypasses the Pi #2 arbiter or the MCU watchdog during normal operation.

## 5. Development stages

### Stage 1: manual open-loop drive

- Pi #2 receives keyboard or joystick input.
- Pi #2 generates left/right PWM targets or a simple velocity command.
- MCU drives both motors and stops on command timeout.
- Encoder signals are observed and validated but need not initially close the loop.

### Stage 2: closed-loop wheel control

- Calibrate encoder polarity and counts.
- Define the Pi-to-MCU target format.
- Implement per-wheel speed control on the MCU.
- Return wheel feedback and derive odometry at the selected layer (`TBD`).

### Stage 3: local assistance

- Pi #2 processes D435 x2 and LiDAR.
- Side cameras operate at a workload appropriate for clearance/obstacle assistance, not as primary SLAM inputs.
- Pi #2 limits speed/turning or stops when local safety conditions require it.

### Stage 4: autonomous navigation

- Pi #1 runs D455-based SLAM/localization.
- Pi #1 and Pi #2 exchange localization state and motion intent.
- The autonomous command source uses the same Pi #2 arbitration path as manual commands.

## 6. Interfaces to define

1. Pi #1 <-> Pi #2 transport, ROS domain/network configuration, messages, QoS, and heartbeat.
2. Pi #2 <-> MCU physical interface and protocol. USB serial is planned but not finalized.
3. Command representation: `geometry_msgs/Twist`, left/right wheel velocity, or another custom message.
4. Feedback ownership: raw ticks, wheel RPM, MCU odometry, and/or Pi #2 odometry.
5. Command publication rate, MCU control rate, watchdog timeout, and stale-data policy.
6. Sensor frames, TF tree, timestamps, synchronization, and calibration procedure.
7. Placement of global planning between Pi #1 and Pi #2 after performance measurement.
8. Hardware and software emergency-stop path.

## 7. Design constraints

- Pi #1 SLAM workload has priority over nonessential processing on that computer.
- Side D435 processing should be limited to the resolution, frame rate, and ROI required for the assistance function; exact values are `TBD` after profiling.
- Loss of a side sensor may permit degraded operation if explicitly allowed by policy.
- Loss of the MCU link or uncontrollable motion must result in a stop.
- System safety must not depend on a single ROS 2 callback continuing to run; the MCU watchdog provides the final software timeout at the drive layer.
