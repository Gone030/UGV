# UGV 하드웨어 통합 테스트 펌웨어

이 타깃은 production controller를 수정하지 않고 RP2040-Zero의 USB CDC, PIO 엔코더, 모터 command 계산, PID, watchdog 및 상태 머신을 단계적으로 검증하기 위한 별도 펌웨어입니다.

## 안전조건

- 부팅 상태는 항상 disable입니다.
- 모드 변경 시 즉시 disable되고 모든 요청값이 0으로 초기화됩니다.
- `ENABLE,1` 전에는 MOTOR/SIM 명령을 적용하지 않습니다.
- 첫 구동 명령 이후 유효한 구동 명령이 1초 동안 없으면 테스트 전용 watchdog이 정지시킵니다.
- `ENABLE,1` 직후에는 watchdog을 시작하지 않고 duty 0으로 명령을 기다립니다.
- production의 `kHardwareOutputEnabled=false`는 유지하며 테스트 실행 파일에서만 PWM/DIR GPIO를 활성화합니다.
- 테스트 모터 출력은 절댓값 기준 최대 10% duty로 제한됩니다.
- ENCODER 모드에서는 항상 motor `stop()`을 호출합니다.
- 테스트용 주기, watchdog, 합성 CPR 및 PID는 이 테스트 실행 파일 안에만 있으며 production 설정값이 아닙니다.

## 테스트 배선

| RP2040-Zero | MDD10A |
|---|---|
| GPIO6 | PWM1 |
| GPIO7 | DIR1 |
| GPIO14 | PWM2 |
| GPIO15 | DIR2 |
| GND | GND |

- PWM 주파수는 20 kHz입니다.
- 모터 전원을 끈 상태에서 신호선과 공통 GND를 먼저 연결합니다.
- 첫 시험은 바퀴 또는 트랙을 지면에서 띄운 상태로 진행합니다.
- 부팅 직후와 모드 변경 직후 PWM duty는 0입니다.

## 빌드

```sh
cd firmware/rp2040_zero/tests/ugv_hw_test
PICO_SDK_PATH=/path/to/pico-sdk cmake -S . -B build -DPICO_BOARD=pico
cmake --build build
```

생성 파일은 `build/ugv_hw_test.uf2`입니다.

Mac 호스트에서 command parser만 테스트할 수 있습니다.

```sh
c++ -std=c++17 -Wall -Wextra -Wpedantic -Iinclude \
  src/command.cpp tests/command_test.cpp -o command_test
./command_test
c++ -std=c++17 -Wall -Wextra -Wpedantic -I../../include \
  ../../src/command_watchdog.cpp ../../src/motor_command.cpp \
  ../../src/system_state.cpp ../../src/velocity_controller.cpp \
  tests/integration_logic_test.cpp -o integration_logic_test
./integration_logic_test
```

## USB CDC 명령

```text
STATUS
MODE,ENCODER
MODE,MOTOR
MODE,SIM
ENABLE,0
ENABLE,1
MOTOR,L,value
MOTOR,R,value
MOTOR,BOTH,left,right
SIM_SPEED,left,right
```

- MOTOR 모드에서 `MOTOR` 값은 `-1.0 ~ +1.0`의 signed command입니다.
- MOTOR 모드의 실제 출력은 명령 크기와 무관하게 `-0.10 ~ +0.10`으로 제한됩니다.
- SIM 모드에서 `MOTOR` 값은 목표 rad/s로 해석됩니다.
- `SIM_SPEED`는 합성 encoder의 피드백 rad/s를 설정합니다.
- `STATUS`는 watchdog heartbeat를 갱신하지 않습니다.

## 출력

- `TEST,<mode>,...`: 테스트 모드, 계산값 및 backend 활성 상태
- `STATE,...`: production 상태 packet serializer 출력
- `ERROR,...`: 명령 오류, 잘못된 모드 또는 watchdog timeout

정상 초기화 후 `motor_hw=1:1`, `encoder_hw=1:1`이 출력됩니다. 모터는
`MODE,MOTOR`, `ENABLE,1` 순서가 완료되고 유효한 `MOTOR` 명령이 들어오기
전까지 duty 0을 유지합니다.
