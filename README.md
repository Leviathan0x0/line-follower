# ESP32-S3 line follower

Firmware for a two-wheel chassis that follows a black tape line on a white surface, including tight curves. It targets an ESP32-S3 DevKit, a 5-channel analog IR array, an L298N, and two N20 gearmotors. Requires ESP-IDF v5.4 or newer.

The robot boots with the brake on. It does not move until you press BOOT or type `go`.

## Power

Use a **2S Li-ion or LiPo, 7.4 V nominal, 1000–2000 mAh, 20C or higher**, and a **5 V buck rated at 2 A or more**.

The L298N drops about 2 V in the motor path. A 7.4 V pack therefore puts about 5.2–5.6 V on a 6 V N20, which is the useful window. A full 8.4 V pack puts about 6.2–6.4 V on the motor. A 4-cell alkaline or NiMH pack is too low after that drop, so curves stall. A 3S pack (11.1 V) puts about 9 V on a 6 V motor and overheats the L298N. This assumes the motors are marked 6 V. If they are marked 3 V, do not use this pack.

| Rail | Source |
| --- | --- |
| Motor supply (`+12V` / VS on the L298N) | Battery positive, through a switch |
| L298N logic 5 V | Buck 5 V. **Remove the 5V-EN jumper** so the onboard 78M05 is out of circuit |
| ESP32-S3 DevKit `5V` pin | The same buck |
| ESP32-S3 `3V3` pin | Sensor VCC only |
| Ground | Battery, buck, L298N, ESP32, and sensors tied together |

Do not power the DevKit from the L298N 5 V pin. That regulator is only good for roughly 100 mA once the motors load the pack. Do not feed the buck into the DevKit `5V` pin while USB is plugged in; on this board those rails meet and the buck can back-feed the computer. Unplug the buck (or open the battery switch) before flashing.

Put a 470 µF electrolytic, 16 V or higher, across the L298N motor supply and ground, close to the driver, and keep the supplied heatsink on the chip. Two stalled N20s can draw 1–2 A.

If you do not want a LiPo, use **six NiMH AA cells (7.2 V, 1900 mAh or more)** and the same 5 V buck. The rest of the wiring does not change.

Optional pack cutoff: set `BOARD_BATTERY_SENSE_ENABLE` to `1` in [main/board_config.h](main/board_config.h). Divider is 22 kΩ from battery positive to GPIO14 and 10 kΩ from GPIO14 to ground. The firmware brakes if the pack stays under 6.6 V. Leave the option at 0 until that divider is fitted. A floating pin will look like a dead battery.

## Wiring

Remove the **ENA** and **ENB** jumpers on the L298N and wire those pins to the ESP32. Leaving a jumper on ties your PWM pin to 5 V.

Sensor index 0 is the far-left channel when the robot faces forward. Power the array from the DevKit `3V3` pin. A 5 V analog output will damage the ESP32-S3 ADC. If a module refuses to run at 3.3 V, power it from the buck and divide every analog output (10 kΩ series, 20 kΩ to ground).

| Signal | GPIO | Connect to |
| --- | --- | --- |
| Sensor 0, far left | 1 | Array analog out 1 |
| Sensor 1 | 2 | Array analog out 2 |
| Sensor 2, center | 4 | Array analog out 3 |
| Sensor 3 | 5 | Array analog out 4 |
| Sensor 4, far right | 6 | Array analog out 5 |
| Left IN1 | 7 | L298N IN1 |
| Left IN2 | 8 | L298N IN2 |
| Left PWM | 9 | L298N ENA |
| Right IN3 | 10 | L298N IN3 |
| Right IN4 | 11 | L298N IN4 |
| Right PWM | 12 | L298N ENB |
| BOOT button | 0 | On the DevKit, active low |
| Status LED | 48 | Onboard WS2812 |

Left motor wires go to OUT1/OUT2. Right motor wires go to OUT3/OUT4. Optional 10 kΩ pull-downs from ENA and ENB to ground keep the motors off during reset, before `app_main` drives the pins.

All tunables and the polarity flags live in [main/board_config.h](main/board_config.h).

## Chassis

The controller is built for a deck about 20 cm long and 10 cm wide.

- Mount the sensor bar on the **front lip**, 8–12 mm above the track, square to the tape. A bar between the wheels sees the curve too late.
- Add a ball caster or two smooth skids. Two wheels alone cannot hold that sensor height.
- Use matte black tape, 15–20 mm wide, on matte white. Gloss and sunlight saturate the IR LEDs. A small card skirt over the sensors helps.
- The line should be continuous. A gap longer than about 3 cm starts a search spin.

## How it follows

Each sensor is normalized from its own white and black calibration, so the code does not assume whether black reads high or low. Samples under the black threshold are ignored. The rest vote in a centroid.

Error is positive when the tape is to the right. The left wheel then speeds up and the right wheel slows down. Past about 80% of full-scale error the cruise speed falls to a crawl and the inner wheel is allowed to reverse, which is what gets a 90° corner around. A short dropout keeps the last command (so a thin tape does not look like a lost line). After that the robot pivots toward the side where the tape was last seen. If it is still lost after 1.4 s it brakes and waits. An all-black reading, such as a crossing, is treated as centered and the robot goes straight.

PWM is 1 kHz, 10-bit, on the L298N enable pins. The bridge is a slow Darlington part; a higher frequency mostly heats it. Commands near zero are braked. Any real command is mapped up to at least `BOARD_MOTOR_START_DUTY` so the gearbox actually starts.

## Build and flash

```bash
. $HOME/esp/esp-idf/export.sh
cd line-follower
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/cu.usbserial-XXXX flash monitor
```

The monitor is 115200 baud. `idf.py set-target esp32s3` has to be run once before the first build. If flashing complains about flash size, set Serial flasher config to the chip on your module (8 MB is the DevKitC-1 default). If the monitor stays blank, the board's USB socket is native USB rather than the UART bridge: set Channel for console output to USB Serial/JTAG.

Hold the chassis in the air for the first power-on. The brake is on, but a wiring mistake should be found with `nudge` before the robot is on the floor.

## Calibrate and drive

On the first boot there is no stored calibration, so the capture starts by itself. Later boots wait 3 seconds. Press BOOT in that window, hold BOOT for 1.2 seconds any time, or type `cal`.

1. Set all five sensors on bare white. Press BOOT. Wait until the log prints the white readings. Do not move the robot during that half second.
2. Hold BOOT and rock the bar sideways across the tape until every sensor has crossed the black. Release BOOT.

The log names any GPIO that did not see enough contrast. Fix that sensor and rock again. A good capture is stored in NVS and survives power cycles.

Short-press BOOT or type `go` to follow. Short-press again, or type `pause`, to brake. The status LED is purple while paused, green while following, amber while searching, and red when the line has been lost for too long or the pack cutoff has tripped.

Type `help` in the monitor. Useful commands:

| Command | Effect |
| --- | --- |
| `show` | Live raw ADC, blackness, position, wheel commands |
| `nudge` | Both wheels forward for 0.4 s |
| `speed 420` | Cruise command, 0–1024 |
| `kp 780` | Steering strength |
| `kd 12` | Damping |
| `ki 40` | Small bias trim |
| `slow 0.55` | How much cruise to shed as the error grows, 0–1 |
| `save` | Store those five values |
| `defaults` | Restore the built-in five |
| `log off` | Silence the 2 Hz line |

`cmd` in the log is the command before the motor start-duty map, not the PWM register.

## Bring-up

Do these in order.

1. `nudge`, with the wheels off the ground. Each wheel must roll the robot forward. If one runs backward, swap that motor's two wires or set `BOARD_MOTOR_LEFT_POLARITY` or `BOARD_MOTOR_RIGHT_POLARITY` to `-1`.
2. `show` on white, then with the center sensor on the tape. Blackness for a covered sensor should rise toward 1000 and the others should stay low. If a channel is stuck at 0 or 4095, check 3.3 V, ground, and that analog pin.
3. `go` on a straight. If it drives off the tape immediately, set `BOARD_SENSOR_REVERSE` to `1` and rebuild. That flips which end of the array is "left".
4. Then tune. Weaving on a straight: lower `kp` or `speed`, or raise `kd` a little. Missing curves: lower `speed`, raise `kp`, raise `slow`, or raise `BOARD_REVERSE_DUTY`.

## Host test

The steering math has no ESP-IDF dependency.

```bash
make -C test test
```

## Layout

| File | Role |
| --- | --- |
| [main/board_config.h](main/board_config.h) | Pins, PID defaults, battery option |
| [main/line_math.c](main/line_math.c) | Centroid, speed mix, lost-line state machine |
| [main/line_sensors.c](main/line_sensors.c) | ADC1 oneshot, per-sensor calibration |
| [main/motors.c](main/motors.c) | L298N direction and 1 kHz PWM |
| [main/follower.c](main/follower.c) | 100 Hz loop, button, LED, battery latch |
| [main/calibrate.c](main/calibrate.c) | White sample and rocked black capture |
| [main/console_ctrl.c](main/console_ctrl.c) | Monitor commands |
