#include "arm_translate_task.h"


static control::MotorCANBase* motor9 = nullptr;
static control::SteeringMotor* base_translate_motor = nullptr;

static bsp::GPIO* base_translate_pe_sensor = nullptr;

bool base_translate_align_detect() {
  return base_translate_pe_sensor->Read() == true;
}


int loop_cnt = 0;
void armTranslateTask(void* arg){
	UNUSED(arg);
	bool aligned = false;
	base_translate_motor->SetMaxSpeed(BASE_TRANSLATE_ALIGN_SPEED);
	while(!aligned){
		base_translate_motor->CalcOutput();
		control::MotorCANBase::TransmitOutput(&motor9, 1);
		aligned = base_translate_motor->Calibrate();

		// print data
		loop_cnt++;
		if(loop_cnt == 100){
			loop_cnt = 0;
			set_cursor(0,0);
			clear_screen();
			base_translate_motor->PrintData();
		}

		osDelay(2);
	}

	// base_translate_motor->ReAlign();
}

void init_arm_translate() {
	// Init m3508 * 1
	motor9 = new control::Motor3508(can1, BASE_TRANSLATE_ID);
	base_translate_pe_sensor = new bsp::GPIO(BASE_TRANSLATE_CALI_GPIO_PORT,
										BASE_TRANSLATE_CALI_GPIO_PIN);
	control::steering_t steering_data;
	steering_data.motor = motor9;
	steering_data.max_speed = BASE_TRANSLATE_RUN_SPEED;
	steering_data.max_acceleration = BASE_TRANSLATE_ACCELERATION;
	// TODO make sure the gear ratio is correct
	steering_data.transmission_ratio = M3508P19_RATIO;
	// steering_data.omega_pid_param = new float[3]{140, 1.2, 25};
	steering_data.omega_pid_param = new float[3]{0, 0, 0};
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