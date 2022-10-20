# 2022 FIRST RAPID REACT

[![CI](https://github.com/strykeforce/rapidreact/actions/workflows/main.yml/badge.svg)](https://github.com/strykeforce/rapidreact/actions/workflows/main.yml)

<p align="center">
  <img alt="skystriker" src="https://live.staticflickr.com/65535/52013666011_a003b55eb0_z.jpg" width="61%">
&nbsp; &nbsp;
  <img alt="skystriker-2" src="https://photos.smugmug.com/photos/i-7bt2T5L/0/L/i-7bt2T5L-L.jpg" width="31%">
</p>

## Controls

![driver](docs/driver-controls.png)
![operator](docs/operator-controls.png)

## Talons

| Subsystem    | Type | Talon         | ID  | PDP | Motor  | Breaker |
| ------------ | ---- | ------------- | --- | --- | ------ | ------- |
| Drive        | SRX  | azimuth       | 0   | 11  | 9015   | 30      |
| Drive        | SRX  | azimuth       | 1   | 18  | 9015   | 30      |
| Drive        | SRX  | azimuth       | 2   | 13  | 9015   | 30      |
| Drive        | SRX  | azimuth       | 3   | 16  | 9015   | 30      |
| Drive        | FX   | drive         | 10  | 12  | Falcon | 40      |
| Drive        | FX   | drive         | 11  | 17  | Falcon | 40      |
| Drive        | FX   | drive         | 12  | 14  | Falcon | 40      |
| Drive        | FX   | drive         | 13  | 15  | Falcon | 40      |
| Intake       | FX   | intake        | 20  | 8   | Falcon | 40      |
| IntakeExtend | SRX  | intakeExtend  | 21  | 2   | 550    | 30      |
| Magazine     | FX   | lowerMagazine | 30  | 9   | Falcon | 30      |
| Magazine     | SRX  | upperMagazine | 31  | 5   | 550    | 30      |
| Shooter      | FX   | shooter       | 40  | 6   | Falcon | 40      |
| Shooter      | FX   | kicker        | 41  | 4   | Falcon | 40      |
| Shooter      | SRX  | hood          | 42  | 7   | 550    | 30      |
| Turret       | FX   | turret        | 50  | 4   | falcon | 30      |
| Climb        | FX   | pivotArm      | 60  | 0   | Falcon | 40      |
| Climb        | FX   | fixedArm      | 61  | 10  | Falcon | 40      |
| Climb        | SRX  | shoulder      | 62  | 1   | 550    | 30      |

* Lower beam break: lower magazine forward limit
* Upper beam break: upper magazine forward limit
* Hood zero sensor : hood forward limit

## Roborio

| Subsystem | Interface | Device      |
| --------- | --------- | ----------- |
| Drive     | SPI/MXP   | NAVX        |
| Magazine  | I2C/MXP   | ColorSensor |

## PWM

| Subsystem | name         | ID |
| --------- | ------------ | -- |
| Climb     | PivotRatchet | 0  |
| Climb     | FixedRatchet | 1  |


## DIO
| Subsystem | name         | ID |
| --------- | ------------ | -- |
| Climb     | leftArmHome  | 0 |
| Climb     | rightArmHome | 1 |
| Auto      | AutoSwitch   | 2 |
| Auto      | AutoSwitch   | 3 |
| Auto      | AutoSwitch   | 4 |
| Auto      | AutoSwitch   | 5 |
| Auto      | AutoSwitch   | 6 |
| Auto      | AutoSwitch   | 7 |
| Robot     | BNC          | 8 |
| Turret    | zeroSensor   | 9 |


## Deadeye Vision System
See the [README](./deadeye/README.md) in `/deadeye`.
