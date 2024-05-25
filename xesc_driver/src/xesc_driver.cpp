#include "xesc_driver/xesc_driver.h"

xesc_driver::XescDriver::XescDriver(ros::NodeHandle &nh, ros::NodeHandle &private_nh) {
    ROS_INFO_STREAM("Starting xesc driver");

    // Check the xesc_type field and create the appropriate driver for it
    std::string xesc_type;
    if(!private_nh.getParam("xesc_type", xesc_type)) {
        ROS_ERROR_STREAM("Error: xesc_type not set.");
        ros::shutdown();
        xesc_driver = nullptr;
        return;
    }

    if(xesc_type == "xesc_2040") {
        xesc_driver = new xesc_2040_driver::Xesc2040Driver(nh, private_nh);
    } else if(xesc_type == "xesc_mini") {
        xesc_driver = new vesc_driver::VescDriver(nh, private_nh);
    } else {
        ROS_ERROR_STREAM("Error: xesc_type invalid. Type was: " << xesc_type);
        ros::shutdown();
        xesc_driver = nullptr;
        return;
    }

    pid_thread_run_ = true;
    pthread_create(&pid_thread_handle_, NULL, &xesc_driver::XescDriver::pid_thread_helper, this);
}

void xesc_driver::XescDriver::getStatus(xesc_msgs::XescStateStamped &state_msg) {
    if (!xesc_driver)
        return;
    xesc_driver->getStatus(state_msg);
}

void xesc_driver::XescDriver::getStatusBlocking(xesc_msgs::XescStateStamped &state_msg) {
    if (!xesc_driver)
        return;
    xesc_driver->getStatusBlocking(state_msg);
}

void xesc_driver::XescDriver::stop() {
    pid_thread_run_ = false;
    if (xesc_driver) xesc_driver->stop();
    pthread_join(pid_thread_handle_, nullptr);
}

void xesc_driver::XescDriver::setDutyCycle(float duty_cycle) {
    if (!xesc_driver)
        return;
    xesc_driver->setDutyCycle(duty_cycle);
}

xesc_driver::XescDriver::~XescDriver() {
    if (xesc_driver) {
        delete xesc_driver;
        xesc_driver = nullptr;
    }
}

void *xesc_driver::XescDriver::pid_thread() {
    float kp, ki, kd = 0.0;
    float err, i_err, d_err, last_err = 0.0;
    ros::Rate rate(100);
    xesc_msgs::XescStateStamped state_msg;

    float dtick, target_dticks = 0.0;
    double last_time = 0;
    float min_dt;
    double dt;
    uint32_t last_tacho = 0;
    float duty = 0.0;
    bool active = false;
    while (pid_thread_run_) {
        rate.sleep();
        ros::Time now = ros::Time::now();
        {
            // Get PID constants and target
            std::unique_lock<std::mutex> lk(pid_config_mutex_);
            kp = this->pid_kp;
            ki = this->pid_ki;
            kd = this->pid_kd;
            min_dt = this->pid_dt;
            target_dticks = this->pid_target;
            active = (now - this->last_pid_target_time).toSec() > 1.0;
        }

        getStatus(state_msg);

        if (!active || last_time == 0) {
            last_err = err = d_err = i_err = 0;
            last_tacho = state_msg.state.tacho_absolute;
            last_time = state_msg.header.stamp.toSec();
            duty = 0;
        }
        else {
            dt = (state_msg.header.stamp.toSec() - last_time);
            if (dt > min_dt) {
                last_time = state_msg.header.stamp.toSec();
                dtick = (state_msg.state.tacho_absolute - last_tacho)/dt;
                last_tacho = state_msg.state.tacho_absolute;
                if(state_msg.state.direction) {
                    dtick *= -1;
                }
                last_err = err;
                err = target_dticks - dtick;
                d_err = (err - last_err)/dt;
                i_err += err/dt;
                float correction = kp * err + ki * i_err + kd * d_err;
                duty = std::min(1.0f, (std::max(-1.0f, duty + correction)));
            }
        }
        this->setDutyCycle(duty);
    }
    return nullptr;
}