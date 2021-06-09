#ifndef __CHASSIS_H__
#define __CHASSIS_H__

class Velocity
{
    public:
        float spd_l;
        float spd_a;
        Velocity(){
        }
        Velocity(const Velocity &vel){
            this->spd_l = vel.spd_l;
            this->spd_a = vel.spd_a;
        }
};

class Odom
{
    public:
        virtual int init() = 0;
        virtual ~Odom() {}
        virtual double get_odo() = 0;
        virtual double get_speed_linear() = 0;
        virtual double get_speed_theta() = 0;
        virtual double get_dist_center() = 0;
        virtual double get_dist_theta() = 0;
        virtual void get_dist(double &, double &) = 0;
};

enum BrakeState
{
    BRAKE_OFF = 0,
    BRAKE_ON = 1
};

class Chassis
{
    private:
        Odom *odom;
    public:
        virtual ~Chassis() {}
        virtual int init() = 0;
        virtual Odom *get_odom() = 0;
        virtual void set_vel(float linear_vel, float angular_vel) = 0;
        virtual int brake(BrakeState state) = 0;
};

#endif
