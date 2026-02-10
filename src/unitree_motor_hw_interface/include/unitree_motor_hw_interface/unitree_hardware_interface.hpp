#ifndef UNITREE_HARDWARE_INTERFACE_HPP
#define UNITREE_HARDWARE_INTERFACE_HPP


#include "hardware_interface/system_interface.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"

#include "unitree_actuator_sdk/serialPort/SerialPort.h"
#include "unitree_actuator_sdk/unitreeMotor/unitreeMotor.h"


namespace unitree_hardware{
    
class UnitreeHardwareInterface : public hardware_interface::SystemInterface
{
public:
    // 生命周期节点接口
    hardware_interface::CallbackReturn
        on_init(const hardware_interface::HardwareInfo & info) override;
    hardware_interface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State & previous_state) override;                      
    hardware_interface::CallbackReturn          
        on_activate(const rclcpp_lifecycle::State & previous_state) override;                       
    hardware_interface::CallbackReturn                                                      
        on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    // 读写接口
    hardware_interface::return_type
        read(const rclcpp::Time & time, const rclcpp::Duration & period) override;    
    hardware_interface::return_type
        write(const rclcpp::Time & time, const rclcpp::Duration & period) override;    

    // 暴露状态和命令接口给Controller Manager
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

private:
    // 端口和设备名称
    std::string serial_port_name_;

    // driver
    std::shared_ptr<SerialPort> serial_ptr_;

    // motor
    std::vector<unsigned short> motor_ids_;
    std::vector<MotorCmd> motor_cmd_vec_;
    std::vector<MotorData> motor_data_vec_;
    double motor_kp;
    double motor_kd;
    float gear_ratios_;                                // 减速比
    
    // ros2_control
    std::vector<double> joint_vel_commands_;
    std::vector<double> joint_pos_commands_;
    std::vector<double> joint_tau_commands_;
    std::vector<double> joint_kp_commands_;
    std::vector<double> joint_kd_commands_;

    std::vector<double> joint_vel_states_;
    std::vector<double> joint_pos_states_;
    std::vector<double> joint_tau_states_;

    // 错误处理
    int communication_err_count_ = 0;
};

}   // namespace unitree_hardware




#endif