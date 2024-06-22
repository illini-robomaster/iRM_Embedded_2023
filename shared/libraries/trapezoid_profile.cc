#include "trapezoid_profile.h"
#include <math.h>
#include "utils.h"

kinematics_state TrapezoidProfile::direct(kinematics_state in)
{
    return {in.position * direction, in.velocity * direction};
}

// update the internal target and return a new target that satisfy the acceleration and velocity limit.
kinematics_state TrapezoidProfile::calculate(float new_target, int current_time_ms)
{

    direction = new_target > current_state.position ? 1 : -1;
    current_state = direct(current_state);
    goal_state = direct(goal_state);
    float t = current_time_ms/1000.0;

    if(current_state.velocity > cruise_velocity) {
        current_state.velocity = cruise_velocity;
    }

    // Deal with a possibly truncated motion profile (with nonzero initial or
    // final velocity) by calculating the parameters as if the profile began and
    // ended at zero velocity
    float cutoffBegin = current_state.velocity / cruise_velocity;
    float cutoffDistBegin = cutoffBegin * cutoffBegin * acceleration / 2.0;

    float cutoffEnd = goal_state.velocity / acceleration;
    float cutoffDistEnd = cutoffEnd * cutoffEnd * acceleration / 2.0;

    // Now we can calculate the parameters as if it was a full trapezoid instead
    // of a truncated one

    float fullTrapezoidDist =
        cutoffDistBegin + (goal_state.position - current_state.position) + cutoffDistEnd;
    float accelerationTime = cruise_velocity / acceleration;

    float fullSpeedDist =
        fullTrapezoidDist - accelerationTime * accelerationTime * acceleration;

    // Handle the case where the profile never reaches full speed
    if (fullSpeedDist < 0) {
      accelerationTime = sqrt(fullTrapezoidDist / acceleration);
      fullSpeedDist = 0;
    }

    end_acc_time = accelerationTime - cutoffBegin;
    end_cruise_time = end_acc_time + fullSpeedDist / cruise_velocity;
    end_decc_time = end_cruise_time + accelerationTime - cutoffEnd;
    kinematics_state result = {current_state.position, current_state.velocity};

    if (t < end_acc_time) {
      result.velocity += t * acceleration;
      result.position += (current_state.velocity + t * acceleration / 2.0) * t;
    } else if (t < end_cruise_time) {
      result.velocity = cruise_velocity;
      result.position +=
          (current_state.velocity + end_acc_time * acceleration / 2.0) * end_acc_time
              + cruise_velocity * (t - end_acc_time);
    } else if (t <= end_decc_time) {
      result.velocity = goal_state.velocity + (end_decc_time - t) * acceleration;
      float timeLeft = end_decc_time - t;
      result.position =
          goal_state.position
              - (goal_state.velocity + timeLeft * acceleration / 2.0) * timeLeft;
    } else {
      result = goal_state;
    }

    return direct(result);
}