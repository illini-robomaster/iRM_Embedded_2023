# Hero 2024 version configuration:

## CAN ID Configurations:

### Gimbal DJI_Board_TypeC:



| Motor Type |     Motor Name      |  CAN   |  TX_ID  |  RX_ID  |
| ---------- | :-----------------: | :----: | :-----: | :-----: |
| Motor4310  |    `load_motor`     | `CAN1` | `0x01`  | `0x02`  |
| Motor3508  | `shoot_front_motor` | `CAN1` | `0x200` | `0x201` |
| Motor3508  | `shoot_back_motor`  | `CAN1` | `0x200` | `0x202` |
| Motor3508  |    `force_motor`    | `CAN1` | `0x200` | `0x203` |
| Motor3508  |    `esca_motor`     | `CAN1` | `0x200` | `0x204` |

## RC Switch Definition:

|        Switch Position        |                           Function                           |
| :---------------------------: | :----------------------------------------------------------: |
|           `swl-up`            |                       Toggle lob mode                        |
| `swr-down (pre-calibration)`  | Calibration, break dead loop and enter primary loops of most threads |
| `swr-down (post-calibration)` |                        Doing nothing                         |
|                               |                                                              |

