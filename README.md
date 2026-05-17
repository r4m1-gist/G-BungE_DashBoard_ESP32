# G-BungE Dashboard ESP32

ESP32-based dashboard firmware for the Mk.5 Baja EV.

## Project Layout

```text
main/
  main.ino

test/
  ledsegmentcheck/
    ledsegmentcheck.ino

  soc_can_display/
    soc_can_display.ino
```

## Sketches

- `main/main.ino`: main dashboard sketch
- `test/ledsegmentcheck/ledsegmentcheck.ino`: 7-segment, LED, and CAN hardware check
- `test/soc_can_display/soc_can_display.ino`: CAN receive/display test

## Notes

The main sketch is currently being built around the Sevcon Gen4 Size 6 controller.
