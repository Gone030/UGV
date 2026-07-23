# UGV Control and State Model

## 1. Purpose

This document separates operational state, control mode, and health. Keeping these axes independent avoids ambiguous states such as treating a noncritical sensor warning as a complete system fault.

The names below are the current design vocabulary. Exact ROS 2 message definitions, transition timeouts, latching behavior, and recovery services are `TBD`.

## 2. Control mode

| Mode | Meaning | Motion source |
|---|---|---|
| `IDLE` | Powered and not accepting a motion request | None; zero target |
| `MANUAL` | Operator-controlled motion | Keyboard, joystick, or external control source through Pi #2 |
| `AUTO` | Autonomous motion | SLAM/navigation stack through Pi #1 and Pi #2 |
| `E_STOP` | Emergency stop active | No motion command accepted |

Entering `E_STOP` immediately invalidates active motion targets. Exit/reset requirements are `TBD`; the conservative default is a latched state requiring an explicit reset after the cause clears.

## 3. System state

| State | Meaning | Motion allowed |
|---|---|---|
| `OFFLINE` | Device/system is powered off or cannot be contacted | No |
| `INITIALIZING` | Boot, communication discovery, and sensor/controller initialization | No |
| `READY` | Required components are available and the system is waiting for activation | Yes, after valid mode/command transition |
| `ACTIVE` | Manual or autonomous drive operation is in progress | Yes |
| `DEGRADED` | A noncritical capability is unavailable but an explicitly limited operating policy remains valid | Conditional |
| `FAULT` | Required capability is unavailable or normal mission execution is unsafe | No |
| `EMERGENCY` | Immediate hazardous condition or emergency-stop condition | No |

`DEGRADED` is not permission to continue by default. Each fault code must explicitly state whether manual motion, reduced-speed motion, or autonomous motion remains permitted.

## 4. Health level

| Level | Meaning | Typical response |
|---|---|---|
| `OK` | Expected operation | None |
| `WARNING` | Abnormal condition with usable capability | Notify, monitor, possibly limit operation |
| `ERROR` | Required function lost or unsafe for normal operation | Stop normal motion and enter `FAULT` unless a defined degraded policy applies |
| `CRITICAL` | Immediate hazard or loss of drive safety | Immediate stop and enter `EMERGENCY` / `E_STOP` |

Health reports should include at least a source, reason code, timestamp, and human-readable detail. Exact message design is `TBD`.

## 5. Command priority

Highest priority first:

```text
1. Emergency stop
2. Safety stop / local collision prevention
3. Manual control
4. Autonomous control
5. Idle / zero command
```

Rules:

- Emergency-stop conditions override every motion source.
- Pi #2 may reduce or reject manual and autonomous commands based on local safety conditions.
- Manual control preempts autonomous control only after a defined, valid mode transition; accidental or stale input must not silently take control.
- Pi #2 publishes only one final command stream to the MCU.
- MCU communication timeout overrides the last received command and produces a stop.
- The timeout duration and whether stop means coast or active brake are `TBD`.

## 6. Ownership of safety decisions

| Condition | Detection owner | Required action |
|---|---|---|
| Operator emergency stop | Operator interface / hardware path TBD | Invalidate command and stop immediately |
| Local obstacle or unsafe clearance | Pi #2 using D435/LiDAR | Limit or stop command according to policy |
| Pi #1 localization/SLAM loss | Pi #1 reports; Pi #2 arbitrates | Stop AUTO; manual/degraded allowance TBD |
| Pi #1 <-> Pi #2 heartbeat loss | Pi #2 | Stop AUTO; manual allowance TBD |
| Pi #2 <-> MCU command loss | MCU watchdog | Stop motors |
| Encoder failure or implausible feedback | MCU, with Pi #2 supervision | Stop affected drive or enter FAULT; detailed policy TBD |
| Battery undervoltage / power fault | Monitoring hardware/software TBD | Warn, limit, or stop based on thresholds TBD |
| Uncommanded motion | MCU/Pi #2 detection TBD | Emergency stop |

## 7. Nominal transitions

```text
OFFLINE
  -> INITIALIZING
  -> READY
  -> ACTIVE
  -> READY

READY or ACTIVE
  -> DEGRADED     when an explicitly tolerated fault occurs
  -> FAULT        when a required capability is lost
  -> EMERGENCY    when immediate stopping is required

DEGRADED
  -> READY/ACTIVE only after the fault clears and the operating policy permits recovery
  -> FAULT or EMERGENCY if the condition worsens

FAULT
  -> INITIALIZING or READY only through a defined reset/recovery procedure

EMERGENCY
  -> INITIALIZING or READY only after the hazard clears and an explicit reset is accepted
```

Automatic recovery from `FAULT` or `EMERGENCY` is not yet approved and remains `TBD`.

## 8. Example composite states

```text
System state : ACTIVE
Control mode : AUTO
Health level : WARNING
Reason       : RIGHT_D435_OFFLINE
```

This may be allowed only if a reduced-capability policy for one missing side camera has been defined.

```text
System state : FAULT
Control mode : IDLE
Health level : ERROR
Reason       : MCU_LINK_LOST
```

The MCU independently stops through its watchdog.

```text
System state : EMERGENCY
Control mode : E_STOP
Health level : CRITICAL
Reason       : UNCOMMANDED_MOTION
```

## 9. Initial implementation scope

For the first manual-drive milestone:

1. Pi #2 accepts a manual input and produces one final drive command.
2. MCU receives the command, drives PWM/direction, and returns basic status.
3. MCU stops when the command becomes stale.
4. Pi #2 exposes at least `IDLE`, `MANUAL`, and `E_STOP` modes.
5. System status exposes at least `INITIALIZING`, `READY`, `ACTIVE`, `FAULT`, and `EMERGENCY`.
6. Encoder inputs are validated before closed-loop speed control is enabled.

The detailed state machine should not be encoded until the following are decided:

- MCU board and Pi-to-MCU protocol.
- Command and feedback message types.
- Heartbeat and watchdog periods.
- Stop/brake behavior.
- E-stop electrical path and reset semantics.
- Which sensor failures permit degraded manual or autonomous motion.
- Battery and power-monitor thresholds.
- Responsibility for odometry computation.
