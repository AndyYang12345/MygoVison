#ifndef GIMBAL_CONTROL_HPP
#define GIMBAL_CONTROL_HPP
#include <iostream>
#include <string>
#include <sstream>
#include <algorithm>
#include <cmath>

class ServoMotor{
public:
    ServoMotor(int id) : _id(id),_angle(0.0f),_speed(0.0f){
        std::cout<<"ServoMotor "<<_id<<" created."<<std::endl;
    };
    ~ServoMotor();
    void set_angle(float angle){
        _angle = angle;
    }
    void add_angle(float delta){
        _angle += delta;
    }
    float get_angle() const{
        return _angle;
    }
    void set_speed(float speed){
        _speed = speed;
    }
    float get_speed() const{
        return _speed;
    }
    void generate_command(float min_angle_deg, float max_angle_deg){
        const float clamped = std::clamp(_angle, min_angle_deg, max_angle_deg);
        _pwm = angle_to_pwm(clamped, min_angle_deg, max_angle_deg);
        if (_speed > 0.0f) {
            _time_ms = static_cast<int>(std::abs(clamped - _last_angle) / _speed * 1000.0f);
        } else {
            _time_ms = 0;
        }
        _last_angle = clamped;

        std::ostringstream oss;
        oss << "#" << _id << "P" << _pwm << "T" << _time_ms << "!";
        _cmd = oss.str();
    }
    const std::string& get_command_buffer() const{
        return _cmd;
    }
private:
    int _id;
    float _angle;
    float _speed;
    float _last_angle{0.0f};
    int _pwm{1500};
    int _time_ms{0};
    std::string _cmd;

    static int angle_to_pwm(float angle_deg, float min_angle_deg, float max_angle_deg){
        const float span = max_angle_deg - min_angle_deg;
        if (span <= 0.0f) return 1500;
        const float t = (angle_deg - min_angle_deg) / span;
        const float pwm = 500.0f + t * (2500.0f - 500.0f);
        return static_cast<int>(std::lround(pwm));
    }
};

class GimbalControl{
public:
    GimbalControl();
    void set_pitch_angle(float angle){
        _pitch_motor.set_angle(angle);
    }
    void set_yaw_angle(float angle){
        _yaw_motor.set_angle(angle);
    }
    void get_command(){
        _pitch_motor.generate_command(0.0f, 180.0f);
        _yaw_motor.generate_command(0.0f, 270.0f);
        _command_buffer = _pitch_motor.get_command_buffer() + _yaw_motor.get_command_buffer();
    }
    const std::string& get_command_buffer() const{
        return _command_buffer;
    }

private:
    ServoMotor _pitch_motor{0};
    ServoMotor _yaw_motor{1};
    std::string _command_buffer;
};

#endif // GIMBAL_CONTROL_HPP