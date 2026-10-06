Assumed pinouts:
 ESP32 CLASSIC

GPIO4 ───────── QTR S0
GPIO16 ───────── QTR S1
GPIO17 ───────── QTR S2
GPIO18 ───────── QTR S3
GPIO19 ───────── QTR S4
GPIO27 ───────── QTR S5
GPIO32 ───────── QTR S6
GPIO33 ───────── QTR S7

GPIO13 ───────── Motor 1 PWM
GPIO14 ───────── Motor 2 PWM
GPIO25 ───────── Motor 3 PWM
GPIO26 ───────── Motor 4 PWM

GPIO21 ───────── SDA ──┬── PCA9685
GPIO22 ───────── SCL ──┼── ToF1
├── ToF2
├── ToF3
└── MPU6050

GPIO1 / GPIO3
└──── keep for USB serial / programming
Along with the PCA9548A.

Always check if the code is aligned to your hardware properly. 
