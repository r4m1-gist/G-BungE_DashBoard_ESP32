# G-BungE Dashboard ESP32

ESP32-based dashboard firmware for the Mk.5 Baja EV.

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
  README.md
```

## Sketches

- `Firmware/main/main.ino`: main dashboard sketch
- `Firmware/test/ledsegmentcheck/ledsegmentcheck.ino`: 7-segment and CAN hardware check
- `Firmware/test/soc_can_display/soc_can_display.ino`: CAN receive/display test

## Hardware

- Controller: NodeMCU ESP32-S
- CAN transceiver: SN65HVD230
- Display: 4-digit 7-segment module x3

## Pin Map

| Part | SCLK | RCLK | DIO |
| --- | --- | --- | --- |
| SEG1 | GPIO21 | GPIO22 | GPIO17 |
| SEG2 | GPIO18 | GPIO19 | GPIO23 |
| SEG3 | GPIO13 | GPIO14 | GPIO25 |

| Part | ESP32 Pin |
| --- | --- |
| CAN TXD / CTX | GPIO26 |
| CAN RXD / CRX | GPIO27 |

## Notes

The main sketch is currently being built around the Sevcon Gen4 Size 6 controller.
