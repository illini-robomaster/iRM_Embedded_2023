#include <chrono>
#include "trapezoid_profile.h"
#include <iostream>
#include <thread>

auto get_current_time_ms(){
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
}

int main(){
    
    std::cout << "Current time in milliseconds is: "
         << get_current_time_ms << std::endl;

    auto start_time = get_current_time_ms();

    // get current time in millesecond

    TrapezoidProfile profile(2, 2, 0);
    kinematics_state last_state = {0,0};
    auto last_loop_time = get_current_time_ms();
    do{
        // get current time
        auto current_time_ms = get_current_time_ms();

        // calculate the new target
        kinematics_state state = profile.calculate(5, current_time_ms-last_loop_time, last_state);
        last_loop_time = current_time_ms;
        std::cout << "time: " << current_time_ms - start_time << " position: " << state.position << " velocity: " << state.velocity << std::endl;
        last_state = state;
        
        // sleep 10ms
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    } while(last_state.position < 5);

}