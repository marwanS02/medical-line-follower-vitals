# Hardware Notes

## Motor Driver (L298N)
- Powered from 7.4V LiPo pack (2S).
- ENA/ENB jumpers removed → controlled by Arduino PWM pins.
- Don’t forget the common GND between Arduino and motor power.

## Motors
- 6V 150 RPM DC gear motors (yellow TT style).
- Left motor slightly faster → added small correction in code.

## IR Line Sensors
- Digital output modules, active LOW on black.
- Sensitivity trimmer: tuned in a dim lab environment.

## MAX30100 Sensor
- Powered from 3.3V pin.
- Placed on servo arm ~2 cm away from chassis edge.

## Bluetooth (HC-05)
- Default baud: 9600.
- RX pin needs a voltage divider to drop 5V → ~3.3V.

## Battery
- 2S LiPo (7.4V nominal) → powers motors via L298N.
- Arduino powered through onboard regulator (Vin).

## Misc
- Servo mounted with hot glue + bracket.
- Chassis is acrylic, 2 layers.
