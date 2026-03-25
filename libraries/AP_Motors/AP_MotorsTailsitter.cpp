/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *       AP_MotorsTailsitter.cpp - ArduCopter motors library for tailsitters and bicopters
 *
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include "AP_MotorsTailsitter.h"
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>
#include <RC_Channel/RC_Channel.h>

extern const AP_HAL::HAL& hal;

#define SERVO_OUTPUT_RANGE  4500

// init
void AP_MotorsTailsitter::init(motor_frame_class frame_class, motor_frame_type frame_type)
{
    // setup default motor and servo mappings
    _has_diff_thrust = SRV_Channels::function_assigned(SRV_Channel::k_motor1) || SRV_Channels::function_assigned(SRV_Channel::k_motor2) || SRV_Channels::function_assigned(SRV_Channel::k_motor3)  || SRV_Channels::function_assigned(SRV_Channel::k_motor4);

    // 四个电机通道 33 34 35 36
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_motor1, CH_1);

    SRV_Channels::set_aux_channel_default(SRV_Channel::k_motor2, CH_2);

    SRV_Channels::set_aux_channel_default(SRV_Channel::k_motor3, CH_3);

    SRV_Channels::set_aux_channel_default(SRV_Channel::k_motor4, CH_4);

    // 机构俯仰倾转舵机 48
    //SRV_Channels::set_aux_channel_default(SRV_Channel::k_tiltpitch, CH_5);
    //SRV_Channels::set_angle(SRV_Channel::k_tiltpitch, SERVO_OUTPUT_RANGE);

    // 机构俯仰倾转舵机 156,157,158,159
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_tiltpitch1, CH_5);
    SRV_Channels::set_angle(SRV_Channel::k_tiltpitch1, SERVO_OUTPUT_RANGE);
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_tiltpitch2, CH_6);
    SRV_Channels::set_angle(SRV_Channel::k_tiltpitch2, SERVO_OUTPUT_RANGE);
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_tiltpitch3, CH_7);
    SRV_Channels::set_angle(SRV_Channel::k_tiltpitch3, SERVO_OUTPUT_RANGE);
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_tiltpitch4, CH_8);
    SRV_Channels::set_angle(SRV_Channel::k_tiltpitch4, SERVO_OUTPUT_RANGE);

    // 机构偏航倾转舵机 49
    //SRV_Channels::set_aux_channel_default(SRV_Channel::k_tiltroll, CH_6);
    //SRV_Channels::set_angle(SRV_Channel::k_tiltroll, SERVO_OUTPUT_RANGE);

    // 机构俯仰倾转舵机 160,161,162,163
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_tiltroll1, CH_9);
    SRV_Channels::set_angle(SRV_Channel::k_tiltroll1, SERVO_OUTPUT_RANGE);
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_tiltroll2, CH_10);
    SRV_Channels::set_angle(SRV_Channel::k_tiltroll2, SERVO_OUTPUT_RANGE);
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_tiltroll3, CH_11);
    SRV_Channels::set_angle(SRV_Channel::k_tiltroll3, SERVO_OUTPUT_RANGE);
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_tiltroll4, CH_12);
    SRV_Channels::set_angle(SRV_Channel::k_tiltroll4, SERVO_OUTPUT_RANGE);

    _mav_type = MAV_TYPE_VTOL_DUOROTOR;

    // record successful initialisation if what we setup was the desired frame_class
    set_initialised_ok(frame_class == MOTOR_FRAME_TAILSITTER);
}


/// Constructor
AP_MotorsTailsitter::AP_MotorsTailsitter(uint16_t speed_hz) :
    AP_MotorsMulticopter(speed_hz)
{
    set_update_rate(speed_hz);
}


// set update rate to motors - a value in hertz
void AP_MotorsTailsitter::set_update_rate(uint16_t speed_hz)
{
    // record requested speed
    _speed_hz = speed_hz;

    SRV_Channels::set_rc_frequency(SRV_Channel::k_motor1, speed_hz);
    SRV_Channels::set_rc_frequency(SRV_Channel::k_motor2, speed_hz);
    SRV_Channels::set_rc_frequency(SRV_Channel::k_motor3, speed_hz);
    SRV_Channels::set_rc_frequency(SRV_Channel::k_motor4, speed_hz);
}

void AP_MotorsTailsitter::output_to_motors()
{
    if (!initialised_ok()) {
        return;
    }

    switch (_spool_state) {
        case SpoolState::SHUT_DOWN:
            SRV_Channels::set_output_pwm(SRV_Channel::k_motor1, get_pwm_output_min());
            SRV_Channels::set_output_pwm(SRV_Channel::k_motor2, get_pwm_output_min());
            SRV_Channels::set_output_pwm(SRV_Channel::k_motor3, get_pwm_output_min());
            SRV_Channels::set_output_pwm(SRV_Channel::k_motor4, get_pwm_output_min());
            break;
        case SpoolState::GROUND_IDLE:
            set_actuator_with_slew(_actuator[1], actuator_spin_up_to_ground_idle());
            SRV_Channels::set_output_pwm(SRV_Channel::k_motor1, output_to_pwm(actuator_spin_up_to_ground_idle()));
            SRV_Channels::set_output_pwm(SRV_Channel::k_motor2, output_to_pwm(actuator_spin_up_to_ground_idle()));
            SRV_Channels::set_output_pwm(SRV_Channel::k_motor3, output_to_pwm(actuator_spin_up_to_ground_idle()));
            SRV_Channels::set_output_pwm(SRV_Channel::k_motor4, output_to_pwm(actuator_spin_up_to_ground_idle()));
            break;
        case SpoolState::SPOOLING_UP:
        case SpoolState::THROTTLE_UNLIMITED:
        case SpoolState::SPOOLING_DOWN:
            SRV_Channels::set_output_pwm(SRV_Channel::k_motor1, output_to_pwm(thr_lin.thrust_to_actuator(_thrust[0])));
            SRV_Channels::set_output_pwm(SRV_Channel::k_motor2, output_to_pwm(thr_lin.thrust_to_actuator(_thrust[1])));
            SRV_Channels::set_output_pwm(SRV_Channel::k_motor3, output_to_pwm(thr_lin.thrust_to_actuator(_thrust[2])));
            SRV_Channels::set_output_pwm(SRV_Channel::k_motor4, output_to_pwm(thr_lin.thrust_to_actuator(_thrust[3])));
            break;
    }

    // use set scaled to allow a different PWM range on plane forward throttle, throttle range is 0 to 100
    //SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, _actuator[2]*100);

    //始终输出给倾转舵机
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltpitch1, _tilt_pitch[0]*SERVO_OUTPUT_RANGE);
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltpitch2, _tilt_pitch[1]*SERVO_OUTPUT_RANGE);
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltpitch3, _tilt_pitch[2]*SERVO_OUTPUT_RANGE);
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltpitch4, _tilt_pitch[3]*SERVO_OUTPUT_RANGE);
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltroll1, _tilt_roll[0]*SERVO_OUTPUT_RANGE);
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltroll2, _tilt_roll[1]*SERVO_OUTPUT_RANGE);
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltroll3, _tilt_roll[2]*SERVO_OUTPUT_RANGE);
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltroll4, _tilt_roll[3]*SERVO_OUTPUT_RANGE);
}

// get_motor_mask - returns a bitmask of which outputs are being used for motors (1 means being used)
//  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
uint32_t AP_MotorsTailsitter::get_motor_mask()
{
    uint32_t motor_mask = 0;
    uint8_t chan;
    if (SRV_Channels::find_channel(SRV_Channel::k_motor1, chan)) {
        motor_mask |= 1U << chan;
    }
    if (SRV_Channels::find_channel(SRV_Channel::k_motor2, chan)) {
        motor_mask |= 1U << chan;
    }
    if (SRV_Channels::find_channel(SRV_Channel::k_motor3, chan)) {
        motor_mask |= 1U << chan;
    }
    if (SRV_Channels::find_channel(SRV_Channel::k_motor4, chan)) {
        motor_mask |= 1U << chan;
    }

    // add parent's mask
    motor_mask |= AP_MotorsMulticopter::get_motor_mask();

    return motor_mask;
}

// calculate outputs to the motors
void AP_MotorsTailsitter::output_armed_stabilizing()
{
    uint8_t i;
    
    float   roll_thrust;                // roll thrust input value, +/- 1.0
    float   pitch_thrust;               // pitch thrust input value, +/- 1.0
    float   yaw_thrust;                 // yaw thrust input value, +/- 1.0
    float   throttle_thrust;            // throttle thrust input value, 0.0 - 1.0
    float   rotate_angle_pitch;               // 手动控制俯仰倾转角
    float   rotate_angle_roll;                // 手动控制滚转倾转角
    float   p_rate;                     // 舵机俯仰控制权重，水平为0，朝下为1，朝上为-1
    float   p_rate_re;                  
    //float   p_rate_abs;
    float   r_rate;                     // 舵机滚转控制权重，水平为0，朝右为1，朝左为-1
    //float   r_rate_abs;
    float   r_rate_re;
    //float   n_rate;
    float   y_rate;                     // 舵机偏航控制权重，取p_rate和r_rate中较大值   
    float   thrust_max;                 // highest motor value
    float   thrust_min;                 // lowest motor value
    float   thr_adj = 0.0f;             // the difference between the pilot's desired throttle and throttle_thrust_best_rpy

    // apply voltage and air pressure compensation
    const float compensation_gain = thr_lin.get_compensation_gain();
    roll_thrust = (_roll_in + _roll_in_ff) * compensation_gain;
    pitch_thrust = _pitch_in + _pitch_in_ff;
    yaw_thrust = _yaw_in + _yaw_in_ff;
    throttle_thrust = get_throttle() * compensation_gain;
    const float max_boost_throttle = _throttle_avg_max * compensation_gain;

    // never boost above max, derived from throttle mix params
    const float min_throttle_out = MIN(_external_min_throttle, max_boost_throttle);
    const float max_throttle_out = _throttle_thrust_max * compensation_gain;

    // 旋转角与俯仰控制权重
    rotate_angle_pitch = RC_Channels::get_radio_in(CH_6);   // 获取遥控器第6通道值
    float rotate_angle_pitch_rad = (rotate_angle_pitch -1515) *0.19f;   // 最大倾斜角度80
    rotate_angle_pitch = rotate_angle_pitch_rad / 80;    // 最大控制角度在AP_AHRS_VIEW和AP_AHRS_DCM中体现,遥控器输入1095——1934

    p_rate = rotate_angle_pitch;

    // 电机俯仰控制权重恒正
    if (p_rate >= 0.0f) 
    {
        p_rate_re = 1.0f - p_rate;
        //p_rate_abs = p_rate;  
        }
    if (p_rate <  0.0f) 
    {
        p_rate_re = 1.0f + p_rate;
        //p_rate_abs = -p_rate;
        }
    
    // 旋转角与偏航控制权重
    rotate_angle_roll = RC_Channels::get_radio_in(CH_5);    // 获取遥控器第5通道值
    float rotate_angle_roll_rad = (rotate_angle_roll -1515) *0.19f;   // 最大倾斜角度80
    rotate_angle_roll = rotate_angle_roll_rad / 80;  // 最大控制角度在AP_AHRS_VIEW和AP_AHRS_DCM中体现,遥控器输入1095——1934
    
    r_rate = rotate_angle_roll;

    if (r_rate >= 0.0f)
    {
        r_rate_re = 1.0f - r_rate;
        //r_rate_abs = r_rate;
    }
    if (r_rate < 0.0f)
    {
        r_rate_re = 1.0f + r_rate;
        //r_rate_abs = - r_rate;
    }
    
    // 偏航系数
    y_rate = MAX(p_rate, r_rate);
    // yaw耦合补偿：大角度时，yaw力矩会耦合到roll/pitch
    // 假设倾转机构在高倾角时，yaw舵机产生额外roll/pitch力矩
    float yaw_pitch_compensation = yaw_thrust * sinf(rotate_angle_pitch_rad) * 0.1f;  // 经验系数，调参
    float yaw_roll_compensation = yaw_thrust * sinf(rotate_angle_roll_rad) * 0.1f;

    // 应用补偿到pitch/roll控制
    pitch_thrust += yaw_pitch_compensation;  // 抵消耦合
    roll_thrust += yaw_roll_compensation;

    // sanity check throttle is above min and below current limited throttle
    if (throttle_thrust <= min_throttle_out) {
        throttle_thrust = min_throttle_out;
        limit.throttle_lower = true;
    }
    if (throttle_thrust >= max_throttle_out) {
        throttle_thrust = max_throttle_out;
        limit.throttle_upper = true;
    }

    if (roll_thrust >= 1.0) {
        // cannot split motor outputs by more than 1
        roll_thrust = 1;
        limit.roll = true;
    }

    // 四个电机推力分配
    _thrust[0]  = throttle_thrust - r_rate_re * roll_thrust * 0.4f + p_rate_re * pitch_thrust * 0.4f + yaw_thrust * 0.13f;
    _thrust[1]  = throttle_thrust + r_rate_re * roll_thrust * 0.4f - p_rate_re * pitch_thrust * 0.4f + yaw_thrust * 0.13f;
    _thrust[2]  = throttle_thrust + r_rate_re * roll_thrust * 0.4f + p_rate_re * pitch_thrust * 0.4f - yaw_thrust * 0.13f;
    _thrust[3]  = throttle_thrust - r_rate_re * roll_thrust * 0.4f - p_rate_re * pitch_thrust * 0.4f - yaw_thrust * 0.13f;

    //gcs().send_text(MAV_SEVERITY_INFO,"%0.2f", pitch_thrust);

    // 四旋翼电机推力限幅
    thrust_min = 1.0f;
    thrust_max = -1.0f;
    for (i = 0; i < 4; i++)
    {
        thrust_max = MAX(_thrust[i], thrust_max);
        thrust_min = MIN(_thrust[i], thrust_min);
    }
    if (thrust_max > 1.0f) {
        // if max thrust is more than one reduce average throttle
        thr_adj = 1.0f - thrust_max;
        limit.throttle_upper = true;
    } else if (thrust_min < 0.0) {
        // if min thrust is less than 0 increase average throttle
        // but never above max boost
        thr_adj = -thrust_min;
        if ((throttle_thrust + thr_adj) > max_boost_throttle) {
            thr_adj = MAX(max_boost_throttle - throttle_thrust, 0.0);
            // in this case we throw away some roll output, it will be uneven
            // constraining the lower motor more than the upper
            // this unbalances torque, but motor torque should have significantly less control power than tilts / control surfaces
            // so its worth keeping the higher roll control power at a minor cost to yaw
            limit.roll = true;
        }
        limit.throttle_lower = true;
    }

    // Add adjustment to reduce average throttle
    for(i = 0; i < 4 ; i++)
    {
    _thrust[i]  = constrain_float(_thrust[i]  + thr_adj, 0.0f, 1.0f);
    }

    _throttle = throttle_thrust;

    // compensation_gain can never be zero
    // ensure accurate representation of average throttle output, this value is used for notch tracking and control surface scaling
    if (_has_diff_thrust) {
        _throttle_out = (throttle_thrust + thr_adj) / compensation_gain;
    } else {
        _throttle_out = throttle_thrust / compensation_gain;
    }

    // 倾转机构控制分配（保留 p_rate / r_rate 权重含义）
    // 基本舵机目标：手动期望角 + 舵机对姿态控制的贡献
    _tilt_pitch[0]  = - rotate_angle_pitch * 0.7f + p_rate * pitch_thrust * 0.5f + y_rate * yaw_thrust * 0.04f;
    _tilt_pitch[1]  = - rotate_angle_pitch * 0.7f - p_rate * pitch_thrust * 0.5f + y_rate * yaw_thrust * 0.04f;
    _tilt_pitch[2]  = - rotate_angle_pitch * 0.7f + p_rate * pitch_thrust * 0.5f - y_rate * yaw_thrust * 0.04f;
    _tilt_pitch[3]  = - rotate_angle_pitch * 0.7f - p_rate * pitch_thrust * 0.5f - y_rate * yaw_thrust * 0.04f;

    // 将一部分偏航映射到滚转舵机（差分）以生成偏航力矩，避免倾斜悬停时仅依赖电机差速
    // 映射方式：左右舵机加减 yaw_servo_component（具体机构映射视机械结构调整）
    _tilt_roll[0] = - rotate_angle_roll * 0.7f - r_rate * roll_thrust * 0.4f - y_rate * yaw_thrust * 0.02f;
    _tilt_roll[1] = - rotate_angle_roll * 0.7f + r_rate * roll_thrust * 0.4f + y_rate * yaw_thrust * 0.02f;  
    _tilt_roll[2] = - rotate_angle_roll * 0.7f + r_rate * roll_thrust * 0.4f - y_rate * yaw_thrust * 0.02f;
    _tilt_roll[3] = - rotate_angle_roll * 0.7f - r_rate * roll_thrust * 0.4f + y_rate * yaw_thrust * 0.02f;
    
    //gcs().send_text(MAV_SEVERITY_INFO,"%0.2f", _tilt_pitch[0]);
    /*_tilt_pitch[0]  = - rotate_angle_pitch * 0.7f;
    _tilt_pitch[1]  = - rotate_angle_pitch * 0.7f;
    _tilt_pitch[2]  = - rotate_angle_pitch * 0.7f;
    _tilt_pitch[3]  = - rotate_angle_pitch * 0.7f;
    _tilt_roll[0] = - rotate_angle_roll * 0.7f;
    _tilt_roll[1] = - rotate_angle_roll * 0.7f;
    _tilt_roll[2] = - rotate_angle_roll * 0.7f;
    _tilt_roll[3] = - rotate_angle_roll * 0.7f;*/
    // 测试舵机变姿旋转极限位置
    //_tilt_pitch  = - rotate_angle_pitch * 0.7f;
    //_tilt_roll = - rotate_angle_roll * 0.7f;

    // 限幅舵机命令，保持在 -1..1
    for (i = 0; i < 4; i++) {
        _tilt_pitch[i] = constrain_float(_tilt_pitch[i], -1.0f, 1.0f);
        _tilt_roll[i]  = constrain_float(_tilt_roll[i], -1.0f, 1.0f);
    }
}

// output_test_seq - spin a motor at the pwm value specified
//  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
//  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
void AP_MotorsTailsitter::_output_test_seq(uint8_t motor_seq, int16_t pwm)
{
    // output to motors and servos
    switch (motor_seq) {
        case 1:
            // right throttle
            SRV_Channels::set_output_pwm(SRV_Channel::k_motor1, pwm);
            break;
        case 2:
            // right tilt servo
            SRV_Channels::set_output_pwm(SRV_Channel::k_motor2, pwm);
            break;
        case 3:
            // left throttle
            SRV_Channels::set_output_pwm(SRV_Channel::k_motor3, pwm);
            break;
        case 4:
            // left tilt servo
            SRV_Channels::set_output_pwm(SRV_Channel::k_motor4, pwm);
            break;
        case 5:
            // left tilt servo
            SRV_Channels::set_output_pwm(SRV_Channel::k_tiltpitch1, pwm);
            break;
        case 6:
            // left tilt servo
            SRV_Channels::set_output_pwm(SRV_Channel::k_tiltpitch2, pwm);
            break;
        case 7:
            // left tilt servo
            SRV_Channels::set_output_pwm(SRV_Channel::k_tiltpitch3, pwm);
            break;
        case 8:
            // left tilt servo
            SRV_Channels::set_output_pwm(SRV_Channel::k_tiltpitch4, pwm);
            break;
        case 9:
            // left tilt servo
            SRV_Channels::set_output_pwm(SRV_Channel::k_tiltroll1, pwm);
            break; 
        case 10:
            // left tilt servo
            SRV_Channels::set_output_pwm(SRV_Channel::k_tiltroll2, pwm);
            break;
        case 11:
            // left tilt servo
            SRV_Channels::set_output_pwm(SRV_Channel::k_tiltroll3, pwm);
            break;
        case 12:
            // left tilt servo
            SRV_Channels::set_output_pwm(SRV_Channel::k_tiltroll4, pwm);
            break; 
        default:
            // do nothing
            break;
    }
}
