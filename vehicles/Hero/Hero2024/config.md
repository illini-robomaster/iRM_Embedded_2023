# Hero 2024 version configuration:

## CAN ID Configurations:

### Gimbal DJI_Board_TypeC:



|  Device Type  |      Defined Name      |  COMM   |     TX_ID      |  RX_ID  |
| :-----------: | :--------------------: | :-----: | :------------: | :-----: |
|   Motor4310   |     `barrel_motor`     | `CAN1`  |     `0x03`     | `0x02`  |
|   Motor4310   |      `yaw_motor`       | `CAN1`  |     `0x05`     | `0x04`  |
|   Motor4310   |   `vtx_pitch_motor`    | `CAN1`  |     `0x07`     | `0x06`  |
|   Motor2006   |    `pitch_motor_L`     | `CAN1`  |   `DEFAULT`    | `0x206` |
|   Motor2006   |    `pitch_motor_R`     | `CAN1`  |   `DEFAULT`    | `0x205` |
|  BRTEncoder   |    `pitch_encoder`     | `CAN1`  |    `NO_TX`     | `0x01`  |
| SEN_0366_DIST |   `distance_sensor`    | `UART1` | `ADDRESS=0x80` |         |
|               |                        |         |                |      


### Shooter DJI_Board_TypeC:


|   Motor3508   |  `shoot_front_motor`   | `CAN1`  |   `DEFAULT`    | `0x201` |
|   Motor3508   |   `shoot_back_motor`   | `CAN1`  |   `DEFAULT`    | `0x202` |
|   Motor3508   |     `force_motor`      | `CAN1`  |   `DEFAULT`    | `0x203` |
|   Motor6020   |      `rfid_motor`      | `CAN1`  |   `DEFAULT`    | `0x209` |
|   Motor3508   |   `escalation_motor`   | `CAN1`  |   `DEFAULT`    | `0x204` |



### Chassis DJI_Board_TypeC

<!-- if one more 6020 motor is on can2 then can bridge will not work -->
| Device Type | Defined Name | COMM   | TX_ID     | RX_ID   |
| :---------: | ------------ | ------ | --------- | ------- |
|  Motor6020  | `motor1`     | `CAN2` | `DEFAULT` | `0x205` |
|  Motor6020  | `motor2`     | `CAN2` | `DEFAULT` | `0x206` |
|  Motor6020  | `motor3`     | `CAN1` | `DEFAULT` | `0x207` |
|  Motor6020  | `motor4`     | `CAN2` | `DEFAULT` | `0x208` | 
|  Motor3508  | `motor5`     | `CAN1` | `DEFAULT` | `0x201` |
|  Motor3508  | `motor6`     | `CAN1` | `DEFAULT` | `0x202` |
|  Motor3508  | `motor7`     | `CAN1` | `DEFAULT` | `0x203` |
|  Motor3508  | `motor8`     | `CAN1` | `DEFAULT` | `0x204` |



## RC Switch Definition:

|        Switch Position        |                           Function                           |
| :---------------------------: | :----------------------------------------------------------: |
|           `swl-up`            |                       Toggle lob mode                        |
|          `swl-down`           |                      Adjust shoot speed                      |
|           `swr-up`            |                       Spin load motor                        |
| `swr-down (pre-calibration)`  | Calibration, break dead loop and enter primary loops of most threads |
| `swr-down (post-calibration)` |  Toggle between main control mode and auxilary control mode  |


## RC Joystick Definition:

### Main control mode:

| Control              | Action                        |
|----------------------|-------------------------------|
| `R horizontal (ch0)` | move chassis left/right       |
| `R vertical (ch1)`   | move chassis forward/backward |
| `L horizontal (ch2)` | rotate chassis ccw/cw         |
| `L vertical (ch3)`   |  pitch camera higher/lower    |

### Auxilary control mode:

| Control               | Action                                      |
|-----------------------|---------------------------------------------|
| `R horizontal (ch0)`  | rotate barrel by one notch (per max stick position) |
| `R vertical (ch1)`    | raise/lower scope (per max stick position)  |
| `L horizontal (ch2)`  | rotate gimbal ccw/cw                        |
| `L vertical (ch3)`    | pitch barrel higher/lower                   |


## Can_Bridge

gimbal id: 0x20A
chassis id: 0x20B
shooter id: 0x20C

Gimbal -> chassis:
    VX, VY, VZ, BUS_SWR, START, DEAD

Gimbal -> shooter:
    START, BUS_SWL, BUS_SWR, DEAD

<!-- The following is ideal but does not work now -->
Chassis -> shooter 
    heat, heat_limit