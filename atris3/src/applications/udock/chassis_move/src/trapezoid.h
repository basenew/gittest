
#ifndef TK_TRAPEZOID_H
#define TK_TRAPEZOID_H

#include <cstdio>
#include <cassert>
#include <vector>
#include <algorithm>
#include <math.h>
#include <iostream>
#include <ros/ros.h>

#define DEGREE_TO_RAD 57.295779513082

struct OneLineSolution{
    double k;
    double t1;
};

struct TwoLineSolution{
    double k;
    double t1;
    double t2;
};

struct ThreeLineSolution{
    double k1;
    double k2;
    double t1;
    double t2;
    double t3;
};

class Rrapezoid
{
private:
    
    // interpolation parameters
    // f(x) = a*x +b
    std::vector<double> m_a,m_b; 
    double m_acc; //accelerator(+)
    double min_velocity;

public:
    std::vector<double> m_x;            // x,y coordinates of points

    Rrapezoid(double acc, double min_vel)
    {
        m_acc = fabs(acc);
        min_velocity = fabs(min_vel);
    }

    bool planning(double velocity_now, double velocity_max, const double position_exp)
    {
        bool res = false;
        struct OneLineSolution one_solution;
        struct TwoLineSolution two_solution;
        struct ThreeLineSolution three_solution;
        m_a.clear();
        m_b.clear();
        m_x.clear();

        if(0 <velocity_max && velocity_max < min_velocity){
            velocity_max = min_velocity;
        }
        else if(velocity_max <0 && velocity_max > -min_velocity){
            velocity_max = -min_velocity;
        }
        if(position_exp >0){
            velocity_max = fabs(velocity_max);
        }
        else{
            velocity_max = -fabs(velocity_max);
        }

        if(velocity_now == velocity_max){
            velocity_now -= 0.01f;
        }

        if(find_one_line_solution(m_acc, velocity_now, position_exp, &one_solution)){
            ROS_INFO_STREAM("one line");
            m_x.push_back(0);
            m_x.push_back(one_solution.t1);

            m_a.push_back(one_solution.k);
            m_b.push_back(velocity_now);
            res = true;
        }
        else if(find_two_line_solution(m_acc, velocity_now, velocity_max, position_exp, &two_solution)){
            ROS_INFO_STREAM("two line");
            m_x.push_back(0);
            m_x.push_back(two_solution.t1);
            m_x.push_back(two_solution.t2);

            m_a.push_back(two_solution.k);
            m_b.push_back(velocity_now);

            m_a.push_back(-two_solution.k);
            m_b.push_back(two_solution.k * two_solution.t2);
            res = true;
        }
        else if(find_three_line_solution(m_acc, velocity_now, velocity_max, position_exp, &three_solution)){
            ROS_INFO_STREAM("three line");
            m_x.push_back(0);
            m_x.push_back(three_solution.t1);
            m_x.push_back(three_solution.t2);
            m_x.push_back(three_solution.t3);
            

            m_a.push_back(three_solution.k1);
            m_b.push_back(velocity_now);

            m_a.push_back(0);
            m_b.push_back(velocity_max);

            m_a.push_back(three_solution.k2);
            m_b.push_back(-three_solution.k2 * three_solution.t3);
            res = true;
        }
        else{
            ROS_INFO_STREAM("no solution");
        }
        return res;
    }

    bool find_one_line_solution(const double a, const double v0, const double s, struct OneLineSolution *solution)
    {
        ROS_INFO_STREAM("try one!" << " [a]" << a << " [v0]" << v0 << " [s]" << s);
        double v0_2 = v0 * v0;
        double as2 = 2 * a * s;
        double eq1_2 = 2 * (v0_2 - as2);
        double eq2_2 = 2 * (v0_2 + as2);
        if(eq1_2 == 0){
            //solution #1
            double k = -a;
            double t1 = v0 / a;
            solution->k = k;
            solution->t1 = t1;
            return true;
        }
        else if(eq2_2 ==0){
            double k = a;
            double t1 = -v0 / a;
            solution->k = k;
            solution->t1 = t1;
            return true;
        }
        return false;
    }

    bool find_two_line_solution(const double a, const double v0, const double vm, const double s, struct TwoLineSolution *solution)
    {
        ROS_INFO_STREAM("try two!" << " [a]" << a << " [v0]" << v0 << " [s]" << s);
        double v0_2 = v0 * v0;
        double as2 = 2 * a * s;
        double eq1_2 = 2 * (v0_2 - as2);
        double eq2_2 = 2 * (v0_2 + as2);

        if(eq1_2 >0){
            //solution #1
            double k = -a;
            double t1 = (2 * v0 + sqrt(eq1_2)) / (2 * a);
            double t2 = (v0 + sqrt(eq1_2)) / a;
            
            ROS_INFO_STREAM("try two 1!" << " t1:" << t1 << " t2:" << t2);
            if(t1 >0 && t2 > t1){
                solution->k = k;
                solution->t1 = t1;
                solution->t2 = t2;
                double vmax = fabs(k * t1 + v0);
                if(vmax < fabs(vm)){
                    return true;
                }
            }
            //solution #2
            k = -a;
            t1 = (2 * v0 - sqrt(eq1_2)) / (2 * a);
            t2 = (v0 - sqrt(eq1_2)) / a;
            ROS_INFO_STREAM("try two 2!" << " t1:" << t1 << " t2:" << t2);
            if(t1 >0 && t2 > t1){
                solution->k = k;
                solution->t1 = t1;
                solution->t2 = t2;
                double vmax = fabs(k * t1 + v0);
                if(vmax < fabs(vm)){
                    return true;
                }
            }
        }
        if(eq2_2 >0){
            //solution #3
            double k = a;
            double t1 = -(2 * v0 + sqrt(eq2_2)) / (2 * a);
            double t2 = -(v0 + sqrt(eq2_2)) / a;
            ROS_INFO_STREAM("try two 3!" << " t1:" << t1 << " t2:" << t2);
            if(t1 >0 && t2 > t1){
                solution->k = k;
                solution->t1 = t1;
                solution->t2 = t2;
                double vmax = fabs(k * t1 + v0);
                if(vmax < fabs(vm)){
                    return true;
                }
            }
            //solution #4
            k = a;
            t1 = -(2 * v0 - sqrt(eq2_2)) / (2 * a);
            t2 = -(v0 - sqrt(eq2_2)) / a;
            ROS_INFO_STREAM("try two 4!" << " t1:" << t1 << " t2:" << t2);
            if(t1 >0 && t2 > t1){
                solution->k = k;
                solution->t1 = t1;
                solution->t2 = t2;
                double vmax = fabs(k * t1 + v0);
                if(vmax < fabs(vm)){
                    return true;
                }
            }
        }
        return false;
    }

    bool find_three_line_solution(const double a, const double v0, const double vm, const double s, struct ThreeLineSolution *solution)
    {
        ROS_INFO_STREAM("try three!" << " [a]" << a << " [v0]" << v0 << " [s]" << s << " [vm]" << vm);
        double v0_2 = v0 * v0;
        double d_vm_2 = 2 *vm * vm;
        double d_a_s = 2 * a *s;
        double d_v0_vm = 2 * v0 *vm;
        double d_a_vm = 2 * a *vm;

        //solution #1
        double k1 = a;
        double k2 = a;
        double t1 = -(v0 - vm) / a;
        double t2 = (v0_2 - d_v0_vm + d_a_s + d_vm_2) / d_a_vm;
        double t3 = (v0_2 - d_v0_vm + d_a_s) / d_a_vm;
        ROS_INFO_STREAM("try three 1!" << " t1:" << t1 << " t2:" << t2 << " t3:" << t3);
        if((t1 >0) && (t2 > t1) && (t3 >t2)){
            solution->k1 = k1;
            solution->k2 = k2;
            solution->t1 = t1;
            solution->t2 = t2;
            solution->t3 = t3;
            return true;
        }
        //solution #2
        k1 = -a;
        k2 = a;
        t1 = (v0 - vm) / a;
        t2 = (-v0_2 + d_v0_vm + d_a_s) / d_a_vm;
        t3 = (-v0_2 + d_v0_vm + d_a_s - d_vm_2) / d_a_vm;
        ROS_INFO_STREAM("try three 2!" << " t1:" << t1 << " t2:" << t2 << " t3:" << t3);
        if((t1 >0) && (t2 > t1) && (t3 >t2)){
            solution->k1 = k1;
            solution->k2 = k2;
            solution->t1 = t1;
            solution->t2 = t2;
            solution->t3 = t3;
            return true;
        }

        //solution #3
        k1 = a;
        k2 = -a;
        t1 = -(v0 - vm) / a;
        t2 = (v0_2 - d_v0_vm + d_a_s) / d_a_vm;
        t3 = (v0_2 - d_v0_vm + d_a_s + d_vm_2) / d_a_vm;
        ROS_INFO_STREAM("try three 3!" << " t1:" << t1 << " t2:" << t2 << " t3:" << t3);
        if((t1 >0) && (t2 > t1) && (t3 >t2)){
            solution->k1 = k1;
            solution->k2 = k2;
            solution->t1 = t1;
            solution->t2 = t2;
            solution->t3 = t3;
            return true;
        }

        //solution #4
        k1 = -a;
        k2 = -a;
        t1 = (v0 - vm) / a;
        t2 = (-v0_2 + d_v0_vm + d_a_s - d_vm_2) / d_a_vm;
        t3 = (-v0_2 + d_v0_vm + d_a_s)  / d_a_vm;
        ROS_INFO_STREAM("try three 4!" << " t1:" << t1 << " t2:" << t2 << " t3:" << t3);
        if((t1 >0) && (t2 > t1) && (t3 >t2)){
            solution->k1 = k1;
            solution->k2 = k2;
            solution->t1 = t1;
            solution->t2 = t2;
            solution->t3 = t3;
            return true;
        }
        // no solution
        return false;
    }

    void operator() (double x, double &speed) const{
        size_t n=m_x.size();
        std::vector<double>::const_iterator it;
        it=std::lower_bound(m_x.begin(),m_x.end(),x);
        int idx=std::max( int(it-m_x.begin())-1, 0);
    
        if(x<m_x[0]) {
            speed= 0;
            std::cout << "erro x0: " << speed << std::endl;
        } else if(x>m_x[n-1]) {
            // extrapolation to the right
            speed= 0;
            // std::cout << "erro x1" << speed << std::endl;
        } else {
            // interpolation
             speed = (m_a[idx])*x+ m_b[idx];
             ROS_INFO_STREAM("id:"<< idx << " x:" << x << " k:" << m_a[idx] << " b:" << m_b[idx]);
        }
    }

};

#endif /*  */
