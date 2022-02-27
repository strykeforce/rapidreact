# 2022 FIRST RAPID REACT

[![CI](https://github.com/strykeforce/rapidreact/actions/workflows/main.yml/badge.svg)](https://github.com/strykeforce/rapidreact/actions/workflows/main.yml)

## Controls

![driver](docs/driver-controls.png)
![operator](docs/operator-controls.png)

## Talons

| Subsystem | Type | Talon           | ID  | PDP | Motor  | Breaker |
| --------- | ---- | --------------- | --- | --- | ------ | ------- |
| Drive     | SRX  | azimuth         | 0   |  11 | 9015   |  30     |
| Drive     | SRX  | azimuth         | 1   |  18 | 9015   |  30     |
| Drive     | SRX  | azimuth         | 2   |  13 | 9015   |  30     |
| Drive     | SRX  | azimuth         | 3   |  16 | 9015   |  30     |
| Drive     | FX   | drive           | 10  |  12 | Falcon |  40     |
| Drive     | FX   | drive           | 11  |  17 | Falcon |  40     |
| Drive     | FX   | drive           | 12  |  14 | Falcon |  40     |
| Drive     | FX   | drive           | 13  |  15 | Falcon |  40     |
| Intake    | FX   | intake          | 20  |  8  | Falcon |  40     |
| Magazine  | SRX  | lowerMagazine   | 30  |  9  | 775    |  30     |
| Magazine  | SRX  | upperMagazine   | 31  |     | 550    |         |
| Shooter   | FX   | shooter         | 40  |     | Falcon |         |
| Shooter   | FX   | kicker          | 41  |     | Falcon |         |
| Shooter   | SRX  | hood            | 42  |     | 550    |         |
| Turret    | SRX  | turret          | 50  | 4   | 550    |         |
| Climb     | FX   | climbExtend1    | 60  | 19  | Falcon |  40     |
| Climb     | FX   | climbExtend2    | 61  | 10  | Falcon |  40     |
| Climb     | SRX  | climbRotate     | 62  |     | 550    |         |

* Lower beam break: lower magazine forward limit
* Upper beam break: upper magazine forward limit
* Turret banana sensor : turret forward limit
* Hood zero sensor : hood forward limit

## Roborio

| Subsystem | Interface | Device      |
| --------- | --------- | ----------- |
| Drive     | SPI/MXP   | NAVX        |
| Magazine  | I2C/MXP   | ColorSensor |

### PWM

| Subsystem | name          | ID |
| --------- | ------------- | -- |
| Climb     | staticRatchet | 0  |
| Climb     | mobileRatchet | 1  |