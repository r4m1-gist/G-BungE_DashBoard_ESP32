이 프로젝트는 마지막 Baja 종목 차량 Mk.5에 들어가는 ESP32 기반 대시보드입니다.

# G-BungE Dashboard ESP32

G-BungE Mk.5 Baja EV 대시보드용 ESP32 펌웨어 및 하드웨어 자료 저장소입니다. Sevcon Gen4 Size 6 컨트롤러의 CAN 데이터를 받아 4-digit 7-segment 모듈에 차량 상태를 표시하는 것을 목표로 합니다.

## Project Layout

```text
Firmware/
  main/
    main.ino

  test/
    ledsegmentcheck/
      ledsegmentcheck.ino

    soc_can_display/
      soc_can_display.ino

Hardware/
  009_GBdashboard.kicad_pcb
  009_GBdashboard.kicad_pro
  009_GBdashboard.kicad_sch
  README.md

  gerber/
    009_GBdashboard.zip
    info.md
```

## Firmware

- `Firmware/main/main.ino`: Mk.5 차량 적용을 위한 메인 대시보드 스케치
- `Firmware/test/ledsegmentcheck/ledsegmentcheck.ino`: 7-segment 모듈 3개와 CAN 송수신 기본 점검용 스케치
- `Firmware/test/soc_can_display/soc_can_display.ino`: CAN으로 수신한 SoC, 온도, 전압 값을 표시하는 테스트 스케치

## Hardware

- Controller: NodeMCU ESP32-S
- CAN transceiver: SN65HVD230
- Display: 4-digit 7-segment module x3
- PCB source: `Hardware/009_GBdashboard.kicad_pcb`, `Hardware/009_GBdashboard.kicad_sch`, `Hardware/009_GBdashboard.kicad_pro`
- Manufacturing export: `Hardware/gerber/009_GBdashboard.zip`

## Pin Map

### 7-Segment Display

| Part | SCLK | RCLK | DIO |
| --- | --- | --- | --- |
| SEG1 | GPIO21 | GPIO22 | GPIO17 |
| SEG2 | GPIO18 | GPIO19 | GPIO23 |
| SEG3 | GPIO13 | GPIO14 | GPIO25 |

### CAN

| Part | ESP32 Pin |
| --- | --- |
| CAN TXD / CTX | GPIO26 |
| CAN RXD / CRX | GPIO27 |

## CAN / Display Behavior

- CAN bus speed: 500 kbps
- Main sketch polls the Sevcon controller over CANopen SDO.
- Display 1 shows motor speed in rpm.
- Display 2 shows motor temperature in degrees Celsius.
- Test sketches are kept for segment wiring checks and CAN receive/display checks before vehicle installation.

## Notes

- Main firmware is built around the Sevcon Gen4 Size 6 controller used in the Mk.5 Baja EV.
- CAN bus termination should be 120 ohm only at each end of the bus.
- Segment display polarity can be adjusted in each sketch with `SEGMENT_ACTIVE_LOW`, `DIGIT_ACTIVE_HIGH`, and `SEND_SEGMENTS_FIRST`.
- Hardware PCB files were added under `Hardware/` for Mk.5 dashboard fabrication and reference.
