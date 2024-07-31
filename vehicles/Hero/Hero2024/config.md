# Hero 2024 version configuration:

## CAN ID Configurations:

### Gimbal DJI_Board_TypeC:



|  Device Type  |      Defined Name      |  COMM   |     TX_ID      |  RX_ID  |
| :-----------: | :--------------------: | :-----: | :------------: | :-----: |
|   Motor4310   |     `barrel_motor`     | `CAN1`  |     `0x03`     | `0x02`  |
|   Motor4310   |      `yaw_motor`       | `CAN1`  |     `0x05`     | `0x04`  |
|   Motor4310   |   `vtx_pitch_motor`    | `CAN1`  |     `0x07`     | `0x06`  |
<!-- |   Motor3508   |   `escalation_motor`   | `CAN1`  |   `DEFAULT`    | `0x204` | -->
|   Motor2006   |    `pitch_motor_L`     | `CAN1`  |   `DEFAULT`    | `0x205` |
|   Motor2006   |    `pitch_motor_R`     | `CAN1`  |   `DEFAULT`    | `0x206` |
|  BRTEncoder   |    `pitch_encoder`     | `CAN1`  |    `NO_TX`     | `0x01`  |
|   Motor3508   |  `shoot_front_motor`   | `CAN2`  |   `DEFAULT`    | `0x201` |
|   Motor3508   |   `shoot_back_motor`   | `CAN2`  |   `DEFAULT`    | `0x202` |
<!-- |   Motor3508   |     `force_motor`      | `CAN2`  |   `DEFAULT`    | `0x203` | -->
|   Motor6020   |      `rfid_motor`      | `CAN2`  |   `DEFAULT`    | `0x209` |
| SEN_0366_DIST |   `distance_sensor`    | `UART1` | `ADDRESS=0x80` |         |
|               |                        |         |                |         |


### Chassis DJI_Board_TypeC

| Device Type | Defined Name | COMM   | TX_ID     | RX_ID   |
| :---------: | ------------ | ------ | --------- | ------- |
|  Motor6020  | `motor1`     | `CAN1` | `DEFAULT` | `0x205` |
|  Motor6020  | `motor2`     | `CAN1` | `DEFAULT` | `0x206` |
|  Motor6020  | `motor3`     | `CAN1` | `DEFAULT` | `0x207` |
|  Motor6020  | `motor4`     | `CAN1` | `DEFAULT` | `0x208` |
|  Motor3508  | `motor5`     | `CAN2` | `DEFAULT` | `0x201` |
|  Motor3508  | `motor6`     | `CAN2` | `DEFAULT` | `0x202` |
|  Motor3508  | `motor7`     | `CAN2` | `DEFAULT` | `0x203` |
|  Motor3508  | `motor8`     | `CAN2` | `DEFAULT` | `0x204` |
|   Motor3508   |   `escalation_motor`   | `CAN1`  |   `DEFAULT`    | `0x204` |
|             |              |        |           |         |



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
| `L vertical (ch3)`   | pitch barrel higher/lower     |

### Auxilary control mode:

| Control               | Action                                      |
|-----------------------|---------------------------------------------|
| `R horizontal (ch0)`  | rotate barrel by one notch (per max stick position) |
| `R vertical (ch1)`    | raise/lower scope (per max stick position)  |
| `L horizontal (ch2)`  | rotate gimbal ccw/cw                        |
| `L vertical (ch3)`    | pitch camera higher/lower                   |