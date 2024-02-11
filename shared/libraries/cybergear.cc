#include "cybergear.h"
#include <cstring>

namespace xiaomi {

/**
  * @brief          提取电机回复帧扩展ID中的电机CANID
  * @param[in]      CAN_ID_Frame:电机回复帧中的扩展CANID
  * @retval         电机CANID
  */
static uint32_t get_motor_id(uint32_t ext_id)
{
  return (ext_id & 0xFFFF) >> 8;
}

void CAN::RxCallback() {
  CAN_RxHeaderTypeDef header;
  uint8_t data[MAX_CAN_DATA_SIZE];
  HAL_CAN_GetRxMessage(hcan_, CAN_RX_FIFO0, &header, data);

  // TODO: ignore broadcast message for now
  if (header.ExtId >> 24 == 0) {
    return;
  }

  uint16_t callback_id = get_motor_id(header.ExtId);
  const auto it = id_to_index_.find(callback_id);
  if (it == id_to_index_.end())
      return;
  callback_id = it->second;
  // find corresponding callback
  if (rx_callbacks_[callback_id])
      rx_callbacks_[callback_id](data, header, rx_args_[callback_id]);
}

// CAN_RxHeaderTypeDef rxMsg;//发送接收结构体
// CAN_TxHeaderTypeDef txMsg;//发送配置结构体
// uint8_t rx_data[8];       //接收数据
// uint32_t Motor_Can_ID;    //接收数据电机ID
// static uint8_t byte[4];          //转换临时数据
// uint32_t send_mail_box = {0};//NONE

/**
  * @brief          浮点数转4字节函数
  * @param[in]      f:浮点数
  * @retval         4字节数组
  * @description  : IEEE 754 协议
  */
// static uint8_t* float_to_byte(float f)
// {
//   unsigned long longdata = 0;
//   memcpy(&f, &longdata, sizeof(float));
//   byte[0] = (longdata & 0xFF000000) >> 24;
//   byte[1] = (longdata & 0x00FF0000) >> 16;
//   byte[2] = (longdata & 0x0000FF00) >> 8;
//   byte[3] = (longdata & 0x000000FF);
//   return byte;
// }

/**
  * @brief          小米电机回文16位数据转浮点
  * @param[in]      x:16位回文
  * @param[in]      x_min:对应参数下限
  * @param[in]      x_max:对应参数上限
  * @param[in]      bits:参数位数
  * @retval         返回浮点值
  */
static float uint16_to_float(uint16_t x, float x_min, float x_max, int bits)
{
    uint32_t span = (1 << bits) - 1;
    float offset = x_max - x_min;
    return offset * x / span + x_min;
}

/**
  * @brief          小米电机发送浮点转16位数据
  * @param[in]      x:浮点
  * @param[in]      x_min:对应参数下限
  * @param[in]      x_max:对应参数上限
  * @param[in]      bits:参数位数
  * @retval         返回浮点值
  */
static int float_to_uint(float x, float x_min, float x_max, int bits)
{
  float span = x_max - x_min;
  float offset = x_min;
  if(x > x_max) x=x_max;
  else if(x < x_min) x= x_min;
  return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}

static void cybergear_callback(const uint8_t data[], const CAN_RxHeaderTypeDef& header, void* args) {
  (void)header;
  CyberGear* motor = reinterpret_cast<CyberGear*>(args);
  motor->UpdateData(data, header);
}

void CyberGear::UpdateData(const uint8_t data[], const CAN_RxHeaderTypeDef& header) {
    angle_=uint16_to_float(data[0]<<8 | data[1], MIN_P, MAX_P, 16);
    speed_=uint16_to_float(data[2]<<8 | data[3], V_MIN, V_MAX, 16);
    torque_=uint16_to_float(data[4]<<8 | data[5], T_MIN, T_MAX, 16);
    temp_=(data[6]<<8|data[7])*Temp_Gain;

    master_can_id_ = header.ExtId & 0xFF;
    error_code_ = (header.ExtId & 0x1F0000)>>16;
}

/**
  * @brief          写入电机参数
  * @param[in]      Motor:对应控制电机结构体
  * @param[in]      index:写入参数对应地址
  * @param[in]      Value:写入参数值
  * @param[in]      Value_type:写入参数数据类型
  * @retval         none
  */
void CyberGear::SetMotorParameter(uint16_t index, float value){
  uint8_t tx_data[8];
  uint32_t ext_id = Communication_Type_SetSingleParameter<<24 | Master_CAN_ID<<8 | this->can_id_;
  tx_data[0]=index;
  tx_data[1]=index>>8;
  tx_data[2]=0x00;
  tx_data[3]=0x00;
  memcpy(tx_data + 4, &value, sizeof(float));
  can_->TransmitExt(ext_id, tx_data, 8);
}

/**
  * @brief          写入电机参数
  * @param[in]      Motor:对应控制电机结构体
  * @param[in]      index:写入参数对应地址
  * @param[in]      Value:写入参数值
  * @param[in]      Value_type:写入参数数据类型
  * @retval         none
  */
void CyberGear::SetMotorParameter(uint16_t index, uint8_t value){
  uint8_t tx_data[8];
  uint32_t ext_id = Communication_Type_SetSingleParameter<<24 | Master_CAN_ID<<8 | this->can_id_;
  tx_data[0]=index;
  tx_data[1]=index>>8;
  tx_data[2]=0x00;
  tx_data[3]=0x00;
  tx_data[4]=value;
  tx_data[5]=0x00;
  tx_data[6]=0x00;
  tx_data[7]=0x00;
  can_->TransmitExt(ext_id, tx_data, 8);
}

/**
  * @brief          小米电机ID检查
  * @param[in]      id:  控制电机CAN_ID【出厂默认0x7F】
  * @retval         none
  */
// void chack_cybergear(uint8_t ID)
// {
//     uint8_t tx_data[8] = {0};
//     txMsg.ExtId = Communication_Type_GetID<<24|Master_CAN_ID<<8|ID;
//     can_txd();
// }

CyberGear::CyberGear(CAN* can, uint8_t can_id, control_mode_t mode) : can_(can), can_id_(can_id) {
  can_->RegisterRxCallback(can_id, cybergear_callback, this);
  SetMode(mode);
  Start();
}

/**
  * @brief          使能小米电机
  * @param[in]      Motor:对应控制电机结构体
  * @retval         none
  */
void CyberGear::Start()
{
    uint8_t tx_data[8] = {0};
    uint32_t ext_id = Communication_Type_MotorEnable<<24|Master_CAN_ID<<8 | can_id_;
    can_->TransmitExt(ext_id, tx_data, 8);
}

/**
  * @brief          停止电机
  * @param[in]      Motor:对应控制电机结构体
  * @param[in]      clear_error:清除错误位（0 不清除 1清除）
  * @retval         None
  */
void CyberGear::Stop(bool clear_error)
{
  uint8_t tx_data[8] = {0};
  tx_data[0] = clear_error;//清除错误位设置
  uint32_t ext_id = Communication_Type_MotorStop<<24|Master_CAN_ID<<8|can_id_;
  can_->TransmitExt(ext_id, tx_data, 8);
}

/**
  * @brief          设置电机模式(必须停止时调整！)
  * @param[in]      Motor:  电机结构体
  * @param[in]      Mode:   电机工作模式（1.运动模式Motion_mode 2. 位置模式Position_mode 3. 速度模式Speed_mode 4. 电流模式Current_mode）
  * @retval         none
  */
void CyberGear::SetMode(const control_mode_t &mode)
{
  SetMotorParameter(Run_mode, (uint8_t)mode);
}

/**
  * @brief          设置电机零点
  * @param[in]      Motor:  电机结构体
  * @retval         none
  */
void CyberGear::SetZeroPosition()
{
  uint8_t tx_data[8]={0};
  tx_data[0] = 1;
  uint32_t ext_id = Communication_Type_SetPosZero<<24|Master_CAN_ID<<8|can_id_;
  can_->TransmitExt(ext_id, tx_data, 8);
}

/**
  * @brief          设置电机CANID
  * @param[in]      Motor:  电机结构体
  * @param[in]      Motor:  设置新ID
  * @retval         none
  */
// void set_CANID_cybergear(MI_Motor *Motor,uint8_t CAN_ID)
// {
//   uint8_t tx_data[8]={0};
//   txMsg.ExtId = Communication_Type_CanID<<24|CAN_ID<<16|Master_CAN_ID<<8|Motor->CAN_ID;
//     Motor->CAN_ID = CAN_ID;//将新的ID导入电机结构体
//     can_txd();
// }

/**
  * @brief          小米运控模式指令
  * @param[in]      Motor:  目标电机结构体
  * @param[in]      torque: 力矩设置[-12,12] N*M
  * @param[in]      position: 位置设置[-12.5,12.5] rad
  * @param[in]      speed: 速度设置[-30,30] rpm
  * @param[in]      kp: 比例参数设置
  * @param[in]      kd: 微分参数设置
  * @retval         none
  */
void CyberGear::SendMotionCommand(float torque, float position, float speed, float kp, float kd)
{
    uint8_t tx_data[8];//发送数据初始化
    //装填发送数据
    tx_data[0]=float_to_uint(position,P_MIN,P_MAX,16)>>8;
    tx_data[1]=float_to_uint(position,P_MIN,P_MAX,16);
    tx_data[2]=float_to_uint(speed,V_MIN,V_MAX,16)>>8;
    tx_data[3]=float_to_uint(speed,V_MIN,V_MAX,16);
    tx_data[4]=float_to_uint(kp,KP_MIN,KP_MAX,16)>>8;
    tx_data[5]=float_to_uint(kp,KP_MIN,KP_MAX,16);
    tx_data[6]=float_to_uint(kd,KD_MIN,KD_MAX,16)>>8;
    tx_data[7]=float_to_uint(kd,KD_MIN,KD_MAX,16);

    uint32_t ext_id = Communication_Type_MotionControl<<24|float_to_uint(torque,T_MIN,T_MAX,16)<<8|can_id_;
    can_->TransmitExt(ext_id, tx_data, 8);
}

void CyberGear::SendCurrentCommand(float current) {
  SetMotorParameter(Iq_Ref, current);
}

float CyberGear::GetAngle() const {
  return angle_;
}

float CyberGear::GetSpeed() const {
  return speed_;
}

float CyberGear::GetTorque() const {
  return torque_;
}

float CyberGear::GetTemperature() const {
  return temp_;
}

uint8_t CyberGear::GetMasterCanID() const {
  return master_can_id_;
}

} // namespace xiaomi
