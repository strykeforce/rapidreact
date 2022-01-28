# 2022 FIRST RAPID REACT

[![CI](https://github.com/strykeforce/rapidreact/actions/workflows/main.yml/badge.svg)](https://github.com/strykeforce/rapidreact/actions/workflows/main.yml)

## Talons

| Subsystem | Type | Talon           | ID  | PDP | Motor  |
| --------- | ---- | --------------- | --- | --- | ------ |
| Drive     | SRX  | azimuth         | 0   |     | 9015   |
| Drive     | SRX  | azimuth         | 1   |     | 9015   |
| Drive     | SRX  | azimuth         | 2   |     | 9015   |
| Drive     | SRX  | azimuth         | 3   |     | 9015   |
| Drive     | FX   | drive           | 10  |     | Falcon |
| Drive     | FX   | drive           | 11  |     | Falcon |
| Drive     | FX   | drive           | 12  |     | Falcon |
| Drive     | FX   | drive           | 13  |     | Falcon |
| Intake    | FX   | intake          | 20  |     | Falcon |
| Magazine  | SRX  | lowerMagazine   | 30  |     | ??     |
| Magazine  | SRX  | upperMagazine   | 31  |     | ??     |
| Shooter   | FX   | shooter         | 40  |     | Falcon |
| Shooter   | SRX  | hood            | 41  |     | ??     |

* Lower beam break: lower magazine forward limit
* Upper beam break: upper magazine forward limit


## Roborio

| Subsystem | Interface | Device      |
| --------- | --------- | ----------- |
| Drive     | SPI/MXP   | NAVX        |
| Magazine  | I2C/MXP   | ColorSensor |

