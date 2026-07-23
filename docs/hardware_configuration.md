# UGV Hardware Configuration

## 1. Document status

This document records the hardware configuration agreed during the initial UGV design discussion. Values that have not been verified or selected are marked `TBD` or `검토 필요`.

## 2. System overview

The platform is a small tracked UGV using two Raspberry Pi 4 Model B computers and one MCU. The two Raspberry Pis divide SLAM and vehicle-control workloads, while the MCU performs low-level motor and encoder handling.

## 3. Computing and sensors

| Group | Component | Quantity | Assigned role | Status |
|---|---|---:|---|---|
| Compute #1 | Raspberry Pi 4 Model B | 1 | Front perception, visual SLAM, pose/map generation | Confirmed |
| Front depth | Intel RealSense D455 | 1 | Front depth input for SLAM | Confirmed |
| Compute #2 | Raspberry Pi 4 Model B | 1 | External/manual command input, side-sensor processing, local safety and command arbitration | Confirmed |
| Side depth | Intel RealSense D435 | 2 | Left/right obstacle and clearance assistance | Confirmed |
| Range sensor | LiDAR | 1 | Obstacle/range sensing for Pi #2 | Model and interface TBD |
| Low-level controller | MCU | 1 | Motor control, encoder acquisition, command timeout stop | Board/model TBD |
| USB distribution | POWERLAN PL-UH305P powered USB 3.0 hub | 1 | Connect D435 x2 and LiDAR to Pi #2 | Confirmed; final modified wiring 검토 필요 |

The PL-UH305P external dimensions are `100 x 45 x 20 mm`. Its original adapter is rated at 5 V, 3 A. The planned supply is the 5 V rail from the D24V90F5. Before permanent modification, PCB polarity and the absence of upstream VBUS backfeed must be verified on the actual unit. If a VBUS-cut upstream cable is used, USB ground, USB 2.0 data, USB 3.x SuperSpeed pairs, and shield continuity must be preserved.

## 4. Drive system

| Component | Quantity | Agreed specification |
|---|---:|---|
| Tracked drive motor | 2 | JGB3865-520R45-12, 12 V, 45:1, 150 +/- 10 RPM, Hall encoder |
| Motor driver | 1 | Cytron MDD10A dual-channel motor driver |

The available motor specification sheet states a 7–13 V operating range, approximately 0.1 A rated current, 1.5 A stall current, and 0.5 N·m stall torque. These values should be verified against the label or supplier documentation for the delivered motors before being used as final electrical or safety limits.

### 4.1 Motor/encoder pinout

The encoder PCB silkscreen was inspected. In the observed connector orientation, the pin order is:

```text
M1
GND
H2
H1
VCC
M2
```

- `M1`, `M2`: motor winding terminals; final polarity depends on the desired installation direction.
- `VCC`, `GND`: Hall encoder supply.
- `H1`, `H2`: quadrature Hall signals; the A/B naming may be assigned in software and direction corrected after testing.
- Encoder supply voltage: begin verification at 3.3 V; final supported range is `TBD` pending authoritative documentation or measurement.
- Encoder counts per revolution and counting convention: `TBD`.

## 5. Power architecture

### 5.1 Source and distribution

- Main battery: 4S LiPo is the current design basis.
- Nominal/full voltage: 14.8 V / 16.8 V.
- Capacity, C-rating, connector gender, main fuse rating, low-voltage cutoff, and emergency disconnect: `TBD`.
- All three converters are connected in parallel to the battery-side distribution point. They are not cascaded from one converter output to another.

```text
4S LiPo
  -> main connector / distribution
     +-> DFRobot DFR1208, 12 V -> Cytron MDD10A -> left/right motors
     +-> Pololu D42V55F5, 5 V -> Pi #1 + front D455
     +-> Pololu D24V90F5, 5 V -> Pi #2 + powered USB hub / side sensors
                                      +-> MCU power through Pi USB (planned)
```

### 5.2 Power converters and loads

| Rail | Converter | Load | Notes |
|---|---|---|---|
| 12 V | DFRobot DFR1208, fixed 12 V / 12 A class | MDD10A and two motors | PCB has no mounting holes; a dedicated insulating protective housing/bracket must be designed. Board envelope: approximately `30 x 26 x 8 mm`. |
| 5 V A | Pololu D42V55F5, 5 V / 6 A class | Pi #1 and D455 | Final Pi input connector/wiring and protection are TBD. |
| 5 V B | Pololu D24V90F5, 5 V / 9 A class | Pi #2, PL-UH305P, D435 x2, LiDAR; MCU via Pi USB | Actual peak load and voltage drop must be measured. |

For both Pololu converters, `EN`, `PG`, and auxiliary pins are not required for basic operation and remain unconnected unless later monitoring/shutdown requirements are defined. Refer to the exact manufacturer datasheet before using these pins.

### 5.3 Grounding and protection

- Battery negative is the common power reference through the converter inputs.
- Signal grounds between Pi #2, MCU, and MDD10A must share a reference.
- High-current motor paths and 5 V computing paths remain separate branches from the main distribution point.
- Fuses, branch protection, reverse-polarity protection strategy, grounding topology, cable gauges in the final harness, and EMI suppression: `검토 필요`.
- Pi undervoltage margin must be checked under simultaneous camera and CPU load.

## 6. Mechanical integration

- MDD10A and Pololu converter CAD models have been identified for mechanical layout.
- DFR1208 requires a custom housing/bracket because the module is supplied as an exposed PCB without mounting holes.
- PL-UH305P may be used without its outer case to save space only after insulation, strain relief, port retention, and PCB mounting are designed and verified.
- CM4/breadboard use was mentioned earlier, but the current confirmed computing configuration is Raspberry Pi 4B x2. Whether a CM4 or small breadboard remains in the final system is `TBD`.

## 7. Open hardware decisions

1. Exact MCU board and logic voltage.
2. Exact LiDAR model, interface, power consumption, and mounting.
3. Battery capacity/C-rating, fuse ratings, cutoff thresholds, and emergency-stop hardware.
4. Encoder voltage range, pulse count, direction convention, and connector type.
5. Final power injection and upstream VBUS/backfeed treatment for the PL-UH305P.
6. Connector family, harness pinout, wire gauges, strain relief, and environmental protection.
7. DFR1208 housing and mounting details.
8. Whether the final compute hardware remains Pi 4B x2 or includes a CM4.
