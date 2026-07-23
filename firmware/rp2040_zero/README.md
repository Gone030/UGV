# UGV RP2040-Zero 저수준 펌웨어 스켈레톤

## 구현 범위와 안전 상태

이 디렉터리에는 ROS를 사용하지 않는 저수준 구동 제어기용 C++/Pico SDK 펌웨어 스켈레톤이 들어 있습니다. USB CDC 줄 단위 프레이밍, 명령 검증, 상태 직렬화, 모터·엔코더·PID·watchdog 인터페이스, 보수적인 상태 머신을 구현합니다. micro-ROS와 Raspberry Pi용 ROS 2 브리지는 포함하지 않습니다.

실제 모터 출력은 기본적으로 비활성화되어 있습니다. GPIO 값은 `-1`, 시간 및 제어기 설정값은 `0` 또는 TBD이며 `kHardwareOutputEnabled`는 `false`입니다. 필요한 설정을 명시적으로 모두 완료하기 전에는 펌웨어가 구동 활성 상태로 진입할 수 없습니다.

## Pi와 RP2040의 책임 분리

- Raspberry Pi: `geometry_msgs/msg/Twist`를 수신하고 차동 궤도형 기구학을 적용한 뒤, 좌우 출력축 목표 각속도를 rad/s 단위로 전송합니다.
- RP2040: 좌우 목표값을 파싱하고, 엔코더 count를 취득해 rad/s를 계산하며, 두 개의 PID를 실행하고, PWM/DIR을 출력하고, 명령 watchdog을 적용한 뒤 피드백과 상태를 반환합니다.
- RP2040은 ROS 2 노드가 아닙니다. Pi와 RP2040은 USB CDC Serial로 통신합니다.

## 빌드 및 UF2 생성

Raspberry Pi Pico SDK와 ARM 임베디드 툴체인을 설치하고 `PICO_SDK_PATH`를 설정합니다.

```sh
cd firmware/rp2040_zero
cmake -S . -B build -DPICO_BOARD=pico
cmake --build build
```

보드 제조사가 별도의 SDK board definition을 제공하지 않는 경우 RP2040-Zero는 일반적으로 표준 RP2040/Pico board definition을 사용합니다. 실제 하드웨어에 적용하기 전에 flash 크기와 board mapping을 반드시 확인해야 합니다. 빌드가 성공하면 `build/ugv_rp2040_zero.uf2`가 생성됩니다. 보드의 BOOT 버튼을 누른 채 연결한 다음, 마운트된 RP2040 USB 대용량 저장 장치에 UF2 파일을 복사합니다.

호스트 전용 테스트는 Pico SDK 없이 빌드할 수 있습니다.

```sh
c++ -std=c++17 -Wall -Wextra -Wpedantic -Iinclude src/protocol.cpp tests/protocol_test.cpp -o protocol_test
./protocol_test
c++ -std=c++17 -Wall -Wextra -Wpedantic -Iinclude \
  src/encoder.cpp src/motor_command.cpp tests/encoder_motor_test.cpp -o encoder_motor_test
./encoder_motor_test
```

## 설정 위치와 TBD 항목

모든 하드웨어 의존 설정은 `include/ugv_mcu/config.hpp`에 있습니다. 다음 항목은 아직 TBD입니다.

- 모터용 GPIO 4개
- 엔코더용 GPIO 4개
- 출력축 1회전당 엔코더 count와 계수 기준
- PWM 주파수
- 제어 주기와 상태 전송 주기
- watchdog timeout
- 좌우 PID 게인과 적분 제한
- 모터 및 엔코더 방향 반전
- 최대 목표 각속도
- 가속도 제한
- coast/brake 정지 방식
- 비상정지 reset 및 해제 조건
- 향후 GPIO interrupt 엔코더 backend 추가 필요 여부

현재 PIO backend에서는 각 엔코더의 B 핀이 A 핀 바로 다음 번호여야 합니다. 즉, `B == A + 1` 조건을 만족해야 합니다. GPIO가 미정이거나 이 조건을 만족하지 않으면 엔코더 backend는 GPIO를 초기화하지 않고 안전 stub로 동작합니다.

## ASCII 프로토콜

각 패킷은 줄바꿈 문자로 끝납니다. 파서는 나뉘어 수신된 입력과 한 번에 연속으로 수신된 여러 줄을 모두 처리합니다. 프로토콜 버전 `1`, sequence 번호, 유한수 여부, enable 값, 필드 개수, 목표값 범위를 검증합니다. 선택 사항인 `*HHHH` CRC-16/CCITT suffix도 지원합니다. 따라서 향후 프로토콜 버전에서 CRC를 필수화하더라도 상위 제어 코드와 wire parser를 직접 결합할 필요가 없습니다.

```text
CMD,1,42,1,3.5,3.5
STATE,1,42,1,0,0,3.5,3.5,3.4,3.6,1024,1031,0.42,0.45
```

형식 오류, NaN/Infinity, 잘못된 버전, 범위 초과 명령은 마지막 유효 명령을 덮어쓰지 않습니다. 이전 명령 내용과 관계없이 watchdog timeout이 발생하면 양쪽 모터 출력을 비활성화하고 두 목표값을 0으로 만듭니다. Sequence freshness 및 replay 처리 정책은 Pi/MCU 통합 단계의 TBD 항목입니다.

## 실모터 연결 전 테스트 순서

1. 호스트 프로토콜 및 encoder/motor 테스트를 실행하고 경계값·fuzz 사례를 추가합니다.
2. 모터 전원을 분리한 상태에서 USB와 상태 전송 관련 설정만 적용하고 CDC 통신을 검증합니다.
3. PIO 엔코더 backend로 각 출력축을 손으로 돌려 방향 부호와 출력축 1회전당 count를 확인합니다.
4. 로직 레벨 관측만 사용해 watchdog timeout과 enable/disable 전이를 검증합니다.
5. PWM/DIR 핀을 지정하되 모터 드라이버를 분리한 상태에서 오실로스코프 또는 로직 애널라이저로 출력을 확인합니다.
6. 물리적 비상정지 경로, 전류 제한, 퓨즈 보호 및 안전한 coast/brake 정책을 확정합니다.
7. 낮은 공급 전압과 전류 제한을 사용하고 한쪽 궤도를 지면에서 띄운 상태에서 시험한 뒤, 방향과 PID를 보수적으로 보정합니다.
8. 앞의 모든 검증을 통과한 뒤에만 `kHardwareOutputEnabled = true`로 변경합니다.
