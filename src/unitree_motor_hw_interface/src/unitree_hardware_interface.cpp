#include "unitree_motor_hw_interface/unitree_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"


namespace unitree_hardware{

hardware_interface::CallbackReturn UnitreeHardwareInterface::on_init
    (const hardware_interface::HardwareInfo & info)
{
    // 1、调用父类初始化
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // 2、获取参数
    try 
    {
        serial_port_name_ = info_.hardware_parameters.at("serial_port");
    } 
    catch (const std::out_of_range &e) 
    {
        RCLCPP_FATAL(rclcpp::get_logger("UnitreeHardwareInterface"), "Parameter 'serial_port' not defined in URDF!");
        return hardware_interface::CallbackReturn::ERROR;
    }
    
    // 3、初始化数据，填充向量
    size_t joint_num = info_.joints.size();
    motor_ids_.resize(joint_num);
    motor_cmd_vec_.resize(joint_num);   // 使用 vector 存储命令
    motor_data_vec_.resize(joint_num);  // 使用 vector 存储反馈

    joint_pos_commands_.resize(joint_num, std::numeric_limits<double>::quiet_NaN());
    joint_vel_commands_.resize(joint_num, 0.0);
    joint_tau_commands_.resize(joint_num, 0.0);
    joint_kp_commands_.resize(joint_num, 0.0);
    joint_kd_commands_.resize(joint_num, 0.0);

    joint_pos_states_.resize(joint_num, 0.0);
    joint_vel_states_.resize(joint_num, 0.0);
    joint_tau_states_.resize(joint_num, 0.0);

    // 4、电机id赋值
    for (size_t i = 0; i < joint_num; i++) 
    {
        try 
        {
            motor_ids_[i] = std::stoi(info_.joints[i].parameters.at("motor_id"));
        } 
        catch (...) 
        {
            RCLCPP_WARN(rclcpp::get_logger("UnitreeHardwareInterface"), 
                "Joint '%s' missing 'motor_id' param, defaulting to %ld", info_.joints[i].name.c_str(), i);
            motor_ids_[i] = i;
        }
        // 预填充 ID 和类型
        motor_cmd_vec_[i].id = motor_ids_[i];
        motor_cmd_vec_[i].motorType = MotorType::GO_M8010_6;       
    }

    // 获取减速比
    gear_ratios_ = queryGearRatio(MotorType::GO_M8010_6);

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn UnitreeHardwareInterface::on_configure
    (const rclcpp_lifecycle::State & /*previous_state*/)
{
    // 初始化硬件
    try
    {
        serial_ptr_ = std::make_shared<SerialPort>(serial_port_name_);
    } 
    catch(...)
    {
        // 初始化失败
        RCLCPP_ERROR(rclcpp::get_logger("UnitreeHardwareInterface"), "Failed to open serial port %s", serial_port_name_.c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("UnitreeHardwareInterface"), "Serial port opened: %s", serial_port_name_.c_str());
    return hardware_interface::CallbackReturn::SUCCESS;
}                      

hardware_interface::CallbackReturn UnitreeHardwareInterface::on_activate
    (const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("UnitreeHardwareInterface"), "Activating motors...");

    // 激活时，读取当前状态并重置命令，防止跳变
    for (size_t i = 0; i < motor_ids_.size(); i++) 
    {
         motor_cmd_vec_[i].mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC);
         motor_cmd_vec_[i].id = motor_ids_[i];
         motor_cmd_vec_[i].q = 0;
         motor_cmd_vec_[i].dq = 0;
         motor_cmd_vec_[i].tau = 0;
         motor_cmd_vec_[i].kp = 0;
         motor_cmd_vec_[i].kd = 0;
    }

    // 增加 try-catch 防止初始化通信异常导致崩溃
    try 
    {
        if(serial_ptr_->sendRecv(motor_cmd_vec_, motor_data_vec_)) 
        {
             for(size_t i=0; i<motor_ids_.size(); i++) 
             {
                 joint_pos_states_[i] = motor_data_vec_[i].q / gear_ratios_;
                 joint_vel_states_[i] = motor_data_vec_[i].dq / gear_ratios_;
                 joint_tau_states_[i] = motor_data_vec_[i].tau * gear_ratios_;
                 
                 joint_pos_commands_[i] = joint_pos_states_[i];
                 joint_vel_commands_[i] = 0.0;
                 joint_tau_commands_[i] = 0.0;
                 joint_kp_commands_[i] = 0.0;
                 joint_kd_commands_[i] = 0.0;
             }
             RCLCPP_INFO(rclcpp::get_logger("UnitreeHardwareInterface"), "Motors initialized successfully.");
        } 
        else 
        {
            RCLCPP_ERROR(rclcpp::get_logger("UnitreeHardwareInterface"), "Failed to communicate with motors on activation!");
            return hardware_interface::CallbackReturn::ERROR;
        }
    } 
    catch (const std::exception &e) 
    {
        RCLCPP_ERROR(rclcpp::get_logger("UnitreeHardwareInterface"), "Exception during activation: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }
    
    return hardware_interface::CallbackReturn::SUCCESS;
}      

hardware_interface::CallbackReturn UnitreeHardwareInterface::on_deactivate
(const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("UnitreeHardwareInterface"), "Deactivating motors (Brake)...");

    for (size_t i = 0; i < motor_ids_.size(); i++) 
    {
        motor_cmd_vec_[i].mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::BRAKE);
        // 【新增】安全清零：确保刹车时不带有残留的力矩或速度指令
        motor_cmd_vec_[i].tau = 0.0;
        motor_cmd_vec_[i].dq = 0.0;
        motor_cmd_vec_[i].kp = 0.0;
        motor_cmd_vec_[i].kd = 0.0;
    }
    try 
    {
        serial_ptr_->sendRecv(motor_cmd_vec_, motor_data_vec_);
    } 
    catch (...) {}

    return hardware_interface::CallbackReturn::SUCCESS;
} 


hardware_interface::return_type UnitreeHardwareInterface::read
    (const rclcpp::Time & time, const rclcpp::Duration & period)
{
    // 由于 sendRecv 是同步收发，我们在 write() 中完成通信以保证指令的新鲜度。
    // read() 这里直接使用 write() 上一次更新的 motor_datas_ 即可。
    // 如果需要更严格的时序，也可以在这里发起查询，但通常在 write 中做闭环控制延迟更低。
    return hardware_interface::return_type::OK;
}   


hardware_interface::return_type UnitreeHardwareInterface::write
    (const rclcpp::Time & time, const rclcpp::Duration & period)
{
    // 1. 准备发送数据
    for (size_t i = 0; i < motor_ids_.size(); i++) 
    {
        motor_cmd_vec_[i].mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC);
        motor_cmd_vec_[i].id = motor_ids_[i];

        // --- 安全性检查 1: NaN 保护 ---
        if (std::isnan(joint_pos_commands_[i])) 
        {
            // 如果指令无效，保持当前位置，并且刚度设为0（阻尼模式）
            motor_cmd_vec_[i].q = joint_pos_states_[i] * gear_ratios_; 
            motor_cmd_vec_[i].kp = 0.0;
            motor_cmd_vec_[i].kd = 0.5; // 给一点阻尼防止自由摆动
            motor_cmd_vec_[i].dq = 0.0;
            motor_cmd_vec_[i].tau = 0.0;
        } 
        else 
        {
            motor_cmd_vec_[i].q = joint_pos_commands_[i] * gear_ratios_;
            motor_cmd_vec_[i].dq = joint_vel_commands_[i] * gear_ratios_;
            motor_cmd_vec_[i].tau = joint_tau_commands_[i] / gear_ratios_;
            
            // --- 安全性检查 2: 默认刚度 ---
            // 如果上层 Controller (如 JointTrajectoryController) 没有下发 kp/kd，默认为 0，电机不会动。
            // 这里给一个保守的默认值，或者完全信任上层。这里示例给一个默认值。
            if (joint_kp_commands_[i] == 0.0 && std::abs(joint_pos_commands_[i]) > 1e-5) {
                motor_cmd_vec_[i].kp = 0.05; // 默认 Kp (需根据实际电机调整!)
                motor_cmd_vec_[i].kd = 0.01; // 默认 Kd
            } 
            else 
            {
                motor_cmd_vec_[i].kp = joint_kp_commands_[i];
                motor_cmd_vec_[i].kd = joint_kd_commands_[i];
            }
        }
    }

    // 2. 批量通信 (增加异常捕获)
    bool comm_success = false;

    try 
    {
        comm_success = serial_ptr_->sendRecv(motor_cmd_vec_, motor_data_vec_);
    } 
    catch (const std::exception &e) 
    {
        // 如果这里抛出异常，不要让整个 controller manager 崩溃
        static rclcpp::Clock exception_clock;
        RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("UnitreeHW"), exception_clock, 1000, 
            "Serial Exception in write(): %s", e.what());
        return hardware_interface::return_type::OK; // 暂时返回OK，避免停机
    }

    if (!comm_success) 
    {
        communication_err_count_++;
        if (communication_err_count_ % 100 == 0) 
        {
            RCLCPP_WARN(rclcpp::get_logger("UnitreeHW"), "Serial communication failed! (Count: %d)", communication_err_count_);
        }
        return hardware_interface::return_type::OK;
    }

    // 3. 解析反馈数据
    for (size_t i = 0; i < motor_ids_.size(); i++) 
    {
        if (motor_data_vec_[i].merror != 0) 
        {
            // 【正确】使用静态 Clock 对象解决左值引用报错
            static rclcpp::Clock throttle_clock;
            RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("UnitreeHW"), throttle_clock, 1000,
                "Motor %d Error: %d (Temp: %d)", motor_ids_[i], motor_data_vec_[i].merror, motor_data_vec_[i].temp);
        }

        joint_pos_states_[i] = motor_data_vec_[i].q / gear_ratios_;
        joint_vel_states_[i] = motor_data_vec_[i].dq / gear_ratios_;
        joint_tau_states_[i] = motor_data_vec_[i].tau * gear_ratios_;
    }

    return hardware_interface::return_type::OK;
}     

std::vector<hardware_interface::StateInterface> UnitreeHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_pos_states_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_vel_states_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &joint_tau_states_[i]));
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> UnitreeHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_pos_commands_[i]));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_vel_commands_[i]));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &joint_tau_commands_[i]));
        
        // 额外暴露 kp 和 kd 接口，可以在 Controller 中通过 names 访问
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, "kp", &joint_kp_commands_[i]));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, "kd", &joint_kd_commands_[i]));
    }
    return command_interfaces;
}

}   // namespace unitree_hardware


// 注册为插件
PLUGINLIB_EXPORT_CLASS(
    unitree_hardware::UnitreeHardwareInterface,                  
    hardware_interface::SystemInterface 
)
