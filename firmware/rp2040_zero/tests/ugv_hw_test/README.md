# UGV 하드웨어 통합 테스트 펌웨어

이 타깃은 production controller를 수정하지 않고 RP2040-Zero의 USB CDC, PIO 엔코더, 모터 command 계산, PID, watchdog 및 상태 머신을 단계적으로 검증하기 위한 별도 펌웨어입니다.

## 안전조건

- 부팅 상태는 항상 disable입니다.
- 모드 변경 시 즉시 disable되고 모든 요청값이 0으로 초기화됩니다.
- `ENABLE,1` 전에는 MOTOR/SIM 명령을 적용하지 않습니다.
- 유효한 구동 명령이 1초 동안 없으면 테스트 전용 watchdog이 정지시킵니다.
- production의 `kHardwareOutputEnabled=false`와 GPIO `-1`을 그대로 사용하므로 실제 PWM/DIR GPIO는 활성화되지 않습니다.
- ENCODER 모드에서는 항상 motor `stop()`을 호출합니다.
- 테스트용 주기, watchdog, 합성 CPR 및 PID는 이 테스트 실행 파일 안에만 있으며 production 설정값이 아닙니다.

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
- SIM 모드에서 `MOTOR` 값은 목표 rad/s로 해석됩니다.
- `SIM_SPEED`는 합성 encoder의 피드백 rad/s를 설정합니다.
- `STATUS`는 watchdog heartbeat를 갱신하지 않습니다.

## 출력

- `TEST,<mode>,...`: 테스트 모드, 계산값 및 backend 활성 상태
- `STATE,...`: production 상태 packet serializer 출력
- `ERROR,...`: 명령 오류, 잘못된 모드 또는 watchdog timeout

현재 설정에서는 `motor_hw=0:0`, `encoder_hw=0:0`, `physical=0`이 정상적인 안전 결과입니다.
