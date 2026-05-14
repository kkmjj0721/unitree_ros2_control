#include "unitree_motor_hw_interface/unitree_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>


namespace unitree_hardware{

namespace
{
constexpr uint32_t kDefaultSerialBaudRate = 4000000;
constexpr int kDefaultStartupRetryCount = 3;
constexpr size_t kDefaultRecvLength = 16;
constexpr const char * kSupportedMotorTypes = "GO_M8010_6, A1, B1";

const char * motor_type_to_string(MotorType motor_type)
{
    switch (motor_type)
    {
        case MotorType::GO_M8010_6:
            return "GO_M8010_6";
        case MotorType::A1:
            return "A1";
        case MotorType::B1:
            return "B1";
    }
    return "UNKNOWN";
}

bool parse_motor_type(const std::string & value, MotorType & motor_type)
{
    if (value == "GO_M8010_6")
    {
        motor_type = MotorType::GO_M8010_6;
        return true;
    }
    if (value == "A1")
    {
        motor_type = MotorType::A1;
        return true;
    }
    if (value == "B1")
    {
        motor_type = MotorType::B1;
        return true;
    }
    return false;
}

bool parse_uint32(const std::string & value, uint32_t & parsed)
{
    try
    {
        if (value.empty() || value[0] == '-')
        {
            return false;
        }
        size_t parsed_chars = 0;
        const unsigned long parsed_long = std::stoul(value, &parsed_chars, 10);
        if (parsed_chars != value.size() ||
            parsed_long > std::numeric_limits<uint32_t>::max())
        {
            return false;
        }
        parsed = static_cast<uint32_t>(parsed_long);
        return true;
    }
    catch (...)
    {
        return false;
    }
}

bool parse_positive_int(const std::string & value, int & parsed)
{
    try
    {
        size_t parsed_chars = 0;
        const long parsed_long = std::stol(value, &parsed_chars, 10);
        if (parsed_chars != value.size() ||
            parsed_long <= 0 ||
            parsed_long > std::numeric_limits<int>::max())
        {
            return false;
        }
        parsed = static_cast<int>(parsed_long);
        return true;
    }
    catch (...)
    {
        return false;
    }
}

bool parse_motor_id(const std::string & value, unsigned short & parsed)
{
    try
    {
        if (value.empty() || value[0] == '-')
        {
            return false;
        }
        size_t parsed_chars = 0;
        const unsigned long parsed_long = std::stoul(value, &parsed_chars, 10);
        if (parsed_chars != value.size() ||
            parsed_long > std::numeric_limits<unsigned short>::max())
        {
            return false;
        }
        parsed = static_cast<unsigned short>(parsed_long);
        return true;
    }
    catch (...)
    {
        return false;
    }
}
}  // namespace

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

    serial_baud_rate_ = kDefaultSerialBaudRate;
    const auto baud_parameter = info_.hardware_parameters.find("serial_baud_rate");
    if (baud_parameter != info_.hardware_parameters.end())
    {
        if (!parse_uint32(baud_parameter->second, serial_baud_rate_) || serial_baud_rate_ == 0)
        {
            RCLCPP_FATAL(rclcpp::get_logger("UnitreeHardwareInterface"),
                "Parameter 'serial_baud_rate' must be a positive uint32 value; got '%s'.",
                baud_parameter->second.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    motor_type_ = MotorType::GO_M8010_6;
    const auto motor_type_parameter = info_.hardware_parameters.find("motor_type");
    if (motor_type_parameter != info_.hardware_parameters.end())
    {
        if (!parse_motor_type(motor_type_parameter->second, motor_type_))
        {
            RCLCPP_FATAL(rclcpp::get_logger("UnitreeHardwareInterface"),
                "Unknown motor_type '%s'. Supported values: %s.",
                motor_type_parameter->second.c_str(), kSupportedMotorTypes);
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    startup_retry_count_ = kDefaultStartupRetryCount;
    const auto retry_parameter = info_.hardware_parameters.find("startup_retry_count");
    if (retry_parameter != info_.hardware_parameters.end())
    {
        if (!parse_positive_int(retry_parameter->second, startup_retry_count_))
        {
            RCLCPP_FATAL(rclcpp::get_logger("UnitreeHardwareInterface"),
                "Parameter 'startup_retry_count' must be a positive integer; got '%s'.",
                retry_parameter->second.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    try
    {
        default_kp_ = std::stod(info_.hardware_parameters.at("default_kp"));
    }
    catch (...)
    {
        default_kp_ = 0.5;
    }

    try
    {
        default_kd_ = std::stod(info_.hardware_parameters.at("default_kd"));
    }
    catch (...)
    {
        default_kd_ = 0.02;
    }

    RCLCPP_INFO(rclcpp::get_logger("UnitreeHardwareInterface"),
        "Hardware config: serial_port=%s baud=%u motor_type=%s startup_retry_count=%d default_kp=%.6f default_kd=%.6f",
        serial_port_name_.c_str(), static_cast<unsigned int>(serial_baud_rate_),
        motor_type_to_string(motor_type_), startup_retry_count_, default_kp_, default_kd_);
    
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
            const auto motor_id_parameter = info_.joints[i].parameters.find("motor_id");
            if (motor_id_parameter == info_.joints[i].parameters.end())
            {
                if (i > std::numeric_limits<unsigned short>::max())
                {
                    RCLCPP_FATAL(rclcpp::get_logger("UnitreeHardwareInterface"),
                        "Joint '%s' index %zu cannot be used as a fallback motor_id.",
                        info_.joints[i].name.c_str(), i);
                    return hardware_interface::CallbackReturn::ERROR;
                }
                RCLCPP_WARN(rclcpp::get_logger("UnitreeHardwareInterface"),
                    "Joint '%s' missing 'motor_id' param, defaulting to %zu",
                    info_.joints[i].name.c_str(), i);
                motor_ids_[i] = static_cast<unsigned short>(i);
            }
            else if (!parse_motor_id(motor_id_parameter->second, motor_ids_[i]))
            {
                RCLCPP_FATAL(rclcpp::get_logger("UnitreeHardwareInterface"),
                    "Joint '%s' has invalid motor_id '%s'; expected unsigned integer 0..%u.",
                    info_.joints[i].name.c_str(), motor_id_parameter->second.c_str(),
                    static_cast<unsigned int>(std::numeric_limits<unsigned short>::max()));
                return hardware_interface::CallbackReturn::ERROR;
            }
        } 
        catch (...) 
        {
            RCLCPP_FATAL(rclcpp::get_logger("UnitreeHardwareInterface"),
                "Exception while parsing motor_id for joint '%s'.", info_.joints[i].name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
        // 预填充 ID 和类型
        motor_cmd_vec_[i].id = motor_ids_[i];
        motor_cmd_vec_[i].motorType = motor_type_;
        motor_data_vec_[i].motorType = motor_type_;
        RCLCPP_INFO(rclcpp::get_logger("UnitreeHardwareInterface"),
            "Joint mapping: %s -> motor_id=%u",
            info_.joints[i].name.c_str(), static_cast<unsigned int>(motor_ids_[i]));
    }

    // 获取减速比
    gear_ratios_ = queryGearRatio(motor_type_);
    RCLCPP_INFO(rclcpp::get_logger("UnitreeHardwareInterface"),
        "Using motor_type=%s gear_ratio=%.6f", motor_type_to_string(motor_type_), gear_ratios_);

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn UnitreeHardwareInterface::on_configure
    (const rclcpp_lifecycle::State & /*previous_state*/)
{
    // 初始化硬件
    try
    {
        RCLCPP_INFO(rclcpp::get_logger("UnitreeHardwareInterface"),
            "Opening serial port %s at %u baud for motor_type=%s",
            serial_port_name_.c_str(), static_cast<unsigned int>(serial_baud_rate_),
            motor_type_to_string(motor_type_));
        serial_ptr_ = std::make_shared<SerialPort>(
            serial_port_name_, kDefaultRecvLength, serial_baud_rate_);
    } 
    catch(const std::exception & e)
    {
        // 初始化失败
        RCLCPP_ERROR(rclcpp::get_logger("UnitreeHardwareInterface"),
            "Failed to open serial port %s at %u baud: %s. Check device path, permissions, baud rate, USB adapter, and whether another process is using the port.",
            serial_port_name_.c_str(), static_cast<unsigned int>(serial_baud_rate_), e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }
    catch(...)
    {
        RCLCPP_ERROR(rclcpp::get_logger("UnitreeHardwareInterface"),
            "Failed to open serial port %s at %u baud. Check device path, permissions, baud rate, USB adapter, and whether another process is using the port.",
            serial_port_name_.c_str(), static_cast<unsigned int>(serial_baud_rate_));
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("UnitreeHardwareInterface"),
        "Serial port opened: %s at %u baud", serial_port_name_.c_str(),
        static_cast<unsigned int>(serial_baud_rate_));
    return hardware_interface::CallbackReturn::SUCCESS;
}                      

hardware_interface::CallbackReturn UnitreeHardwareInterface::on_activate
    (const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("UnitreeHardwareInterface"),
        "Activating motors: serial_port=%s baud=%u motor_type=%s startup_retry_count=%d",
        serial_port_name_.c_str(), static_cast<unsigned int>(serial_baud_rate_),
        motor_type_to_string(motor_type_), startup_retry_count_);

    if (!serial_ptr_)
    {
        RCLCPP_ERROR(rclcpp::get_logger("UnitreeHardwareInterface"), "Serial port is not configured before activation.");
        return hardware_interface::CallbackReturn::ERROR;
    }

    // 激活时，读取当前状态并重置命令，防止跳变
    for (size_t i = 0; i < motor_ids_.size(); i++)
    {
         RCLCPP_INFO(rclcpp::get_logger("UnitreeHardwareInterface"),
            "Activation target: %s -> motor_id=%u",
            info_.joints[i].name.c_str(), static_cast<unsigned int>(motor_ids_[i]));
         motor_cmd_vec_[i].mode = queryMotorMode(motor_type_, MotorMode::FOC);
         motor_cmd_vec_[i].id = motor_ids_[i];
         motor_cmd_vec_[i].motorType = motor_type_;
         motor_data_vec_[i].motorType = motor_type_;
         motor_cmd_vec_[i].q = 0;
         motor_cmd_vec_[i].dq = 0;
         motor_cmd_vec_[i].tau = 0;
         motor_cmd_vec_[i].kp = 0;
         motor_cmd_vec_[i].kd = 0;
    }

    // 逐个读取启动状态。这样一个电机通信失败时，日志会明确指出具体 joint/motor_id。
    try
    {
        for(size_t i=0; i<motor_ids_.size(); i++)
        {
            bool comm_success = false;
            for (int attempt = 1; attempt <= startup_retry_count_; ++attempt)
            {
                comm_success = serial_ptr_->sendRecv(&motor_cmd_vec_[i], &motor_data_vec_[i]);
                if (comm_success)
                {
                    break;
                }
                RCLCPP_WARN(rclcpp::get_logger("UnitreeHardwareInterface"),
                    "Startup read retry %d/%d failed for joint '%s' motor_id=%u on %s at %u baud (motor_type=%s)",
                    attempt, startup_retry_count_, info_.joints[i].name.c_str(),
                    static_cast<unsigned int>(motor_ids_[i]), serial_port_name_.c_str(),
                    static_cast<unsigned int>(serial_baud_rate_), motor_type_to_string(motor_type_));
            }

            if (!comm_success)
            {
                RCLCPP_ERROR(rclcpp::get_logger("UnitreeHardwareInterface"),
                    "Activation aborted: joint '%s' motor_id=%u did not reply after %d startup read attempt(s) on %s at %u baud using motor_type=%s. Check motor power, motor_id mapping, wiring, baud rate, serial permissions, and that no other process is using the adapter.",
                    info_.joints[i].name.c_str(), static_cast<unsigned int>(motor_ids_[i]),
                    startup_retry_count_, serial_port_name_.c_str(),
                    static_cast<unsigned int>(serial_baud_rate_), motor_type_to_string(motor_type_));
                return hardware_interface::CallbackReturn::ERROR;
            }

            const double joint_position = motor_data_vec_[i].q / gear_ratios_;
            const double joint_velocity = motor_data_vec_[i].dq / gear_ratios_;
            const double joint_effort = motor_data_vec_[i].tau * gear_ratios_;

            // Startup state is the true current motor feedback scaled by gear ratio only.
            // Calibration, sign, and zero offsets are intentionally not applied yet.
            joint_pos_states_[i] = joint_position;
            joint_vel_states_[i] = joint_velocity;
            joint_tau_states_[i] = joint_effort;

            joint_pos_commands_[i] = joint_pos_states_[i];
            joint_vel_commands_[i] = 0.0;
            joint_tau_commands_[i] = 0.0;
            joint_kp_commands_[i] = 0.0;
            joint_kd_commands_[i] = 0.0;

            RCLCPP_INFO(rclcpp::get_logger("UnitreeHardwareInterface"),
                "Startup joint '%s' motor_id=%u raw_q=%.6f pos=%.6f vel=%.6f effort=%.6f",
                info_.joints[i].name.c_str(), static_cast<unsigned int>(motor_ids_[i]), motor_data_vec_[i].q,
                joint_position, joint_velocity, joint_effort);
        }

        RCLCPP_WARN(rclcpp::get_logger("UnitreeHardwareInterface"),
            "Startup states use raw motor q/gear feedback only; calibration, sign, and zero offsets are not implemented yet.");
        RCLCPP_INFO(rclcpp::get_logger("UnitreeHardwareInterface"), "Motors initialized successfully.");
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
        motor_cmd_vec_[i].mode = queryMotorMode(motor_type_, MotorMode::BRAKE);
        motor_cmd_vec_[i].motorType = motor_type_;
        motor_data_vec_[i].motorType = motor_type_;
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
    (void)time;
    (void)period;

    // 由于 sendRecv 是同步收发，我们在 write() 中完成周期通信以保证指令的新鲜度。
    // read() 这里直接使用 on_activate() 或 write() 上一次更新的 motor_data_vec_ 状态。
    // 当前状态是 raw motor q/gear 结果；校准、方向符号和零位偏置尚未实现。
    // 如果需要更严格的时序，也可以在这里发起查询，但通常在 write 中做闭环控制延迟更低。
    return hardware_interface::return_type::OK;
}   


hardware_interface::return_type UnitreeHardwareInterface::write
    (const rclcpp::Time & time, const rclcpp::Duration & period)
{
    (void)time;
    (void)period;

    // 1. 准备发送数据
    for (size_t i = 0; i < motor_ids_.size(); i++) 
    {
        motor_cmd_vec_[i].mode = queryMotorMode(motor_type_, MotorMode::FOC);
        motor_cmd_vec_[i].id = motor_ids_[i];
        motor_cmd_vec_[i].motorType = motor_type_;
        motor_data_vec_[i].motorType = motor_type_;

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
            
            // 如果上层 Controller 没有下发 kp/kd，就使用 hardware 默认刚度。
            // 注意：即使目标位置是 0，也必须给 kp/kd，否则 reset_zero 末端会变成无刚度命令。
            motor_cmd_vec_[i].kp = (std::abs(joint_kp_commands_[i]) > 1e-9)
                ? joint_kp_commands_[i]
                : default_kp_;
            motor_cmd_vec_[i].kd = (std::abs(joint_kd_commands_[i]) > 1e-9)
                ? joint_kd_commands_[i]
                : default_kd_;
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
            "Serial Exception in write() on %s at %u baud (motor_type=%s): %s",
            serial_port_name_.c_str(), static_cast<unsigned int>(serial_baud_rate_),
            motor_type_to_string(motor_type_), e.what());
        return hardware_interface::return_type::ERROR;
    }

    if (!comm_success) 
    {
        communication_err_count_++;
        RCLCPP_ERROR(rclcpp::get_logger("UnitreeHW"),
            "Serial sendRecv failed in write() on %s at %u baud (motor_type=%s); returning ERROR to stop/flag hardware. Count: %d",
            serial_port_name_.c_str(), static_cast<unsigned int>(serial_baud_rate_),
            motor_type_to_string(motor_type_), communication_err_count_);
        return hardware_interface::return_type::ERROR;
    }

    // 3. 解析反馈数据
    bool motor_error_reported = false;
    for (size_t i = 0; i < motor_ids_.size(); i++) 
    {
        if (motor_data_vec_[i].merror != 0) 
        {
            RCLCPP_ERROR(rclcpp::get_logger("UnitreeHW"),
                "Motor fault reported in write(): joint '%s' motor_id=%u motor_type=%s merror=%d temp=%d; returning ERROR",
                info_.joints[i].name.c_str(), static_cast<unsigned int>(motor_ids_[i]),
                motor_type_to_string(motor_type_), motor_data_vec_[i].merror, motor_data_vec_[i].temp);
            motor_error_reported = true;
        }
    }

    if (motor_error_reported)
    {
        return hardware_interface::return_type::ERROR;
    }

    for (size_t i = 0; i < motor_ids_.size(); i++)
    {
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
