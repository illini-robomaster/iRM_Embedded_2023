#include "arm_translate_task.h"

static control::MotorCANBase* motor9 = nullptr;
static control::SteeringMotor* base_translate_motor = nullptr;

static bsp::GPIO* base_translate_pe_sensor = nullptr;

bool base_translate_align_detect() {
  return base_translate_pe_sensor->Read() == false; // only the white wire for the light gate works, which needs to be reversed
}


int loop_cnt = 0;
void armTranslateTask(void* arg){
	UNUSED(arg);
	bool aligned = false;
        print("Starting Calib\r\n");
	base_translate_motor->SetMaxSpeed(BASE_TRANSLATE_ALIGN_SPEED);
	while(!aligned){
		base_translate_motor->CalcOutput();
		control::MotorCANBase::TransmitOutput(&motor9, 1);
		aligned = base_translate_motor->Calibrate();

		// print data
		loop_cnt++;
		if(loop_cnt == 100) {
                  loop_cnt = 0;
                  set_cursor(0, 0);
                  clear_screen();
//                  print("aligned: %d .. \r\n", base_translate_motor->Calibrate());
                  print("HAL tick: %d \r\n", HAL_GetTick());
                  base_translate_motor->PrintData();
                  motor9->PrintData();
                }
		osDelay(2);
	}
        base_translate_motor->ReAlign();
        base_translate_motor->TurnRelative(0);
        base_translate_motor->SetMaxSpeed(BASE_TRANSLATE_RUN_SPEED);
        control::MotorCANBase::TransmitOutput(&motor9, 1);


        while(1) {
          loop_cnt++;
          if(loop_cnt >= 100) {
            set_cursor(0, 0);
            clear_screen();
            base_translate_motor->PrintData();
            motor9->PrintData();
            loop_cnt = 0;
            print("dbus ch2: %f \r\n", dbus->ch3/660.0/200);

          }


          base_translate_motor->TurnRelative(dbus->ch3 / 660.0 / 50);
          base_translate_motor->CalcOutput();
          control::MotorCANBase::TransmitOutput(&motor9, 1);
          osDelay(2);
        }
}


// map entire range to one rotation
int TRANSLATE_RATIO = 4; // full translation range is this many rotation
void init_arm_translate() {
	// Init m3508 * 1
	motor9 = new control::Motor3508(can1, BASE_TRANSLATE_ID);
	base_translate_pe_sensor = new bsp::GPIO(IN1_GPIO_Port, IN1_Pin);
	control::steering_t steering_data;
	steering_data.motor = motor9;
	steering_data.max_speed = BASE_TRANSLATE_RUN_SPEED;
	steering_data.max_acceleration = BASE_TRANSLATE_ACCELERATION;
	// TODO make sure the gear ratio is correct
	steering_data.transmission_ratio = M3508P19_RATIO /** TRANSLATE_RATIO*/;
    steering_data.omega_pid_param = new float[3]{140, 1.2, 25};
//	steering_data.omega_pid_param = new float[3]{0, 0, 0};
	steering_data.max_iout = 1000;
	steering_data.max_out = 13000;
	// TODO measure the calibrate offset for base translate motor
	steering_data.calibrate_offset = 0;
	steering_data.align_detect_func = base_translate_align_detect;

	base_translate_motor = new control::SteeringMotor(steering_data);
}

void kill_arm_translate(){
	RM_EXPECT_TRUE(false, "Operation Killed!\r\n");

	while(true){
		motor9->SetOutput(0);
		control::MotorCANBase::TransmitOutput(&motor9, 1);

		osDelay(100);
	}	

}