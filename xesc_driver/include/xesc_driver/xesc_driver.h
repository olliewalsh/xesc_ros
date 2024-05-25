//
// Created by clemens on 03.07.22.
//

#ifndef SRC_XESC_DRIVER_H
#define SRC_XESC_DRIVER_H

#include <pthread.h>
#include <mutex>

#include <ros/ros.h>
#include <xesc_interface/xesc_interface.h>
#include "xesc_2040_driver/xesc_2040_driver.h"
#include "vesc_driver/vesc_driver.h"


namespace xesc_driver  {
    class XescDriver: public xesc_interface::XescInterface {
    public:
        XescDriver(ros::NodeHandle &nh, ros::NodeHandle &private_nh);
        ~XescDriver();

        void getStatus(xesc_msgs::XescStateStamped &state) override;

        void getStatusBlocking(xesc_msgs::XescStateStamped &state) override;

        void setDutyCycle(float duty_cycle) override;

        void stop() override;

        void setSpeed(float ticks_per_second) {
            {
                std::unique_lock<std::mutex> lk(pid_config_mutex_);
                pid_target = ticks_per_second;
                last_pid_target_time = ros::Time::now();
            }
        }

        void setPID(float kp, float ki, float kd, float dt) {
            {
                std::unique_lock<std::mutex> lk(pid_config_mutex_);
                pid_kp = kp;
                pid_ki = ki;
                pid_kd = kd;
                pid_dt = dt;
            }
        }

    private:
        xesc_interface::XescInterface *xesc_driver = nullptr;

        static void *pid_thread_helper(void *context) {
            return ((xesc_driver::XescDriver *) context)->pid_thread();
        }
        void *pid_thread();
        pthread_t pid_thread_handle_;
        bool pid_thread_run_;
        std::mutex pid_config_mutex_;
        float pid_kp;
        float pid_ki;
        float pid_kd;
        float pid_target;
        float pid_dt;
        ros::Time last_pid_target_time;
    };
}

#endif //SRC_XESC_DRIVER_H
