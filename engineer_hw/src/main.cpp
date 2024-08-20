#include "Serial.h"
#include "rclcpp/rclcpp.hpp"
#include <vector>
#include "engineer_msg/msg/end_effector.hpp"
#include "engineer_msg/msg/sucket_state.hpp"
#include "engineer_msg/msg/lift_state.hpp"
#include "engineer_msg/msg/joint_state.hpp"
#include "engineer_msg/msg/joint_command.hpp"
#include "engineer_msg/msg/robot_arm_state.hpp"
#include "engineer_msg/srv/fsm_command.hpp"
#include "engineer_msg/srv/fsm_state.hpp"
#include "CRC.h"
#include <cmath>

static WzSerialPort serial;

std::string now_fsm_state;

//extern class std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("engineer_hw");

float arm_motor_real[6];
float arm_motor_target[6] = {0, 0, 0, 0, 0, 0};
float pre_arm_motor_target[6] = {};
bool sucket_target = false;
bool lift_rise = false, lift_fall = false;
uint8_t auto_action_code;
bool robotarm_ready = false;
uint8_t arm_alive_status = false;

void send_target_angle_to_motor(){
    uint8_t tx_buf[33];
    tx_buf[0] = 0xAB;
    //使用memcpy复制到缓冲区
    memcpy(tx_buf + 1, arm_motor_target, 24);
    tx_buf[25] = ((uint8_t)sucket_target | ((uint8_t)lift_rise << 1) | ((uint8_t)lift_fall << 2) | ((uint8_t)robotarm_ready << 4));
    tx_buf[26] = 0xff;
    if (now_fsm_state == "custom_control")
    {
        tx_buf[25] |= (0x01 << 3);
        tx_buf[26] = 0x00;
    }
    else if (now_fsm_state == "auto_perlace_1")
    {
        tx_buf[25] &= ~(0x01 << 3);
        tx_buf[26] = 0x01;
    }
    else if (now_fsm_state == "auto_perlace_2")
    {
        tx_buf[25] &= ~(0x01 << 3);
        tx_buf[26] = 0x02;
    }
    else if (now_fsm_state == "auto_takeout_1")
    {
        tx_buf[25] &= ~(0x01 << 3);
        tx_buf[26] = 0x03;
    }
    else if (now_fsm_state == "auto_takeout_2")
    {
        tx_buf[25] &= ~(0x01 << 3);
        tx_buf[26] = 0x04;
    }
    else if (now_fsm_state == "little_source_land")
    {
        tx_buf[25] &= ~(0x01 << 3);
        tx_buf[26] = 0x05;
    }
    else if (now_fsm_state == "big_source_land")
    {
        tx_buf[25] |= (0x01 << 3);
        tx_buf[26] = 0x07;
    }
    else if (now_fsm_state == "defend")
    {
        tx_buf[25] |= (0x01 << 3);
        tx_buf[26] = 0x08;
    }
    
    
    
    
    uint16_t data_CRC16 = Get_CRC16_Check_Sum(tx_buf, 31, 0xFFFF);
    tx_buf[31] = data_CRC16 & 0xFF;
    tx_buf[32] = (data_CRC16 >> 8) & 0xFF;
    serial.send(tx_buf, 33);
}

//电机目标角度接收回调函数
void joint_command_callback(const engineer_msg::msg::JointCommand::SharedPtr msg) {
    //将接收到的指令复制到arm_motor_target
    for (int i = 0; i < 6; i++) {
        arm_motor_target[i] = msg->position[i];
    }
}

//机械臂自动动作接收回调函数
void auto_command_callback(const engineer_msg::msg::RobotArmState::SharedPtr msg) {
    auto_action_code = msg->auto_action;
}

//吸盘目标状态接收回调函数
void sucket_command_callback(const engineer_msg::msg::SucketState::SharedPtr msg) {
    //将接收到的指令复制到sucket_target
    sucket_target = msg->state;
}

//升降状态接收回调函数
void lift_command_callback(const engineer_msg::msg::LiftState::SharedPtr msg) {
    //接收指令
    lift_rise = msg->rise;
    lift_fall = msg->fall;
    // RCLCPP_INFO(node->get_logger(), "%d", lift_rise);
}

//角度归一化
double NormAngle(double angle){
    float a = std::fmod(angle + M_PI, 2.0 * M_PI);
    if (a < 0.0)
    {
        a += 2.0 * M_PI;
    }
    return a - M_PI;
}

void FSMState_Callback(rclcpp::Client<engineer_msg::srv::FSMState>::SharedFuture response)
{
    // 使用response的get()获取
    auto result = response.get();
    now_fsm_state = result->status_now;
}

int main(){
    if (!serial.open("/dev/ttyEngineer", 115200, 0, 8, 1)) {
        RCLCPP_ERROR(rclcpp::get_logger("engineer_hw"), "cannot open serial port");
        // return -1;
    }

    rclcpp::init(0, nullptr);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("engineer_hw");

    //初始化一个ROS2 publisher来发布当前每一个电机的角度
    auto joint_state_publisher = node->create_publisher<engineer_msg::msg::JointState>(
        "joint_state", 10);

    //初始化一个ROS2 publisher来发布当前机械臂状态
    auto robotarm_state_publisher = node->create_publisher<engineer_msg::msg::RobotArmState>(
        "robotarm_state", 10);

    //初始化一个ROS2 subscriber来接收控制指令
    auto joint_command_subscriber = node->create_subscription<engineer_msg::msg::JointCommand>(
        "joint_command", 10, joint_command_callback);

    //初始化一个ROS2 subscriber来接收机械臂动作指令
    auto auto_command_subscriber = node->create_subscription<engineer_msg::msg::RobotArmState>(
        "auto_command", 10, auto_command_callback);

    //初始化一个ROS2 subscriber来接收吸盘状态
    auto sucket_command_subscriber = node->create_subscription<engineer_msg::msg::SucketState>(
        "sucket_command", 10, sucket_command_callback);

    //初始化一个ROS2 subscriber来接收升降状态
    auto lift_command_subscriber = node->create_subscription<engineer_msg::msg::LiftState>(
        "lift_command", 10, lift_command_callback);

    //初始化一个ROS2 Client来请求机械臂运动
    auto fsm_command_client = node->create_client<engineer_msg::srv::FSMCommand>("fsm_command");

    //初始化一个ROS2 Client来请求机械臂fsm状态
    auto fsm_state_client = node->create_client<engineer_msg::srv::FSMState>("fsm_state");

    const int rx_buf_len = 33;
    uint8_t rx_buf[rx_buf_len];
    int rx_len = 0, rx_recv_len = 0;
    int error_cnt = 0;
    uint8_t fsm_command = 0x00, pre_fsm_command = 0x00;
    float position_error = 0.0f, error_tmp = 0.0f;
    float position_alter = 0.0f, alter_tmp = 0.0f;

    while (rclcpp::ok())
    {
        // find frame head
        uint16_t Pack_len;
        //获取时间
        auto start = time(NULL);
        auto now = time(NULL);
        auto diff = now - start;
        while(serial.receive(rx_buf, 1) == 0){
            now = time(NULL);
            diff = now - start;
            if (diff >= 2)
            {
                break;
            }
        }
        if (diff >= 2)
        {
            //重新尝试打开串口
            RCLCPP_INFO(node->get_logger(), "No message in %d s, try to reopen serial.", diff);
            // serial.close();
            if (!serial.open("/dev/ttyEngineer", 115200, 0, 8, 1)) {
                RCLCPP_ERROR(rclcpp::get_logger("customctrl_serial_communication"), "cannot open serial port");
            }
            continue;
        }


        if (rx_buf[0] != 0xAC) {
            // 16进制打印每个字节
            std::cout << std::hex << (int)rx_buf[0] << std::endl;
            continue;
        }
        rx_recv_len = 1;
        while (rx_recv_len < rx_buf_len) {
            while((rx_len = serial.receive(rx_buf + rx_recv_len, rx_buf_len - rx_recv_len)) == 0);
            rx_recv_len += rx_len;
        }

        // RCLCPP_INFO(node->get_logger(),"%d", auto_prepare_code);

        //将缓冲区的数据复制到arm_motor_real
        //从将缓冲区的数据复制到arm_motor_real[1]开始
        //因为缓冲区的第一个字节是0xAC，不是电机的数据
        //使用memcpy
        memcpy(arm_motor_real, rx_buf + 1, 24);

        // ROS2 JointState message
        engineer_msg::msg::JointState joint_state_msg;
        joint_state_msg.header.stamp = rclcpp::Clock().now();
        joint_state_msg.header.frame_id = "engineer_arm";
        //Set current position of six joints
        position_error = 0.0f;
        position_alter = 0.0f;
        arm_alive_status = rx_buf[25] & 0x01;
        if (!arm_alive_status)
        {
            RCLCPP_INFO(node->get_logger(), "arm not alive!!");
        }
        
        for(int i = 0; i < 6; i++) {
            if (now_fsm_state != "defend")
            {
                if ((arm_motor_target[i] - pre_arm_motor_target[i]) < -3.1415926f)
                {
                    arm_motor_target[i] += 3.1415926f * 2.0f;
                }
                else if ((arm_motor_target[i] - pre_arm_motor_target[i]) > 3.1415926f)
                {
                    arm_motor_target[i] -= 3.1415926f * 2.0f;
                }
            }
            
            
            arm_motor_target[5] = NormAngle(arm_motor_target[5]);

            joint_state_msg.position[i] = arm_motor_real[i];

            //计算机械臂实际位置与目标位置的误差
            error_tmp = arm_motor_target[i] - arm_motor_real[i];
            position_error += error_tmp * error_tmp;

            alter_tmp = pre_arm_motor_target[i] - arm_motor_target[i];
            position_alter += alter_tmp * alter_tmp;
            pre_arm_motor_target[i] = arm_motor_target[i];
            // RCLCPP_INFO(node->get_logger(), "motor: %d, %f, %f", i, error_tmp, alter_tmp);
        }
        
        joint_state_msg.lift_position = rx_buf[27];
        //先不管速度和力矩，直接发送消息
        joint_state_publisher->publish(joint_state_msg);

        if (position_alter >= 0.2f || (!arm_alive_status))
        {
            //当输出位置出现跳变时，机械臂进入慢速状态
            robotarm_ready = false;
        }
        else if (!robotarm_ready && (position_error <= 0.0001) && arm_alive_status)
        {
            //当位置误差足够小时，机械臂准备就绪
            robotarm_ready = true;
        }
        if(!robotarm_ready){
            // RCLCPP_INFO(node->get_logger(), "arm not ready, : %f, %f",position_error, position_alter);
        }

        //RCLCPP_INFO(node->get_logger(), "error:%f, alter:%f, %f", position_error, position_alter, arm_motor_target[6]);

        /* 请求机械臂运动服务 */
        fsm_command = rx_buf[26];
        auto fsm_command_request = std::make_shared<engineer_msg::srv::FSMCommand::Request>();
        auto fsm_state_request = std::make_shared<engineer_msg::srv::FSMState::Request>();
        // RCLCPP_INFO(node->get_logger(), "%d", rx_buf[26]);

        engineer_msg::msg::RobotArmState robotarm_state_msg;
        //auto_action_code不为0时为自动化模式，原则上不能切换状态
        if (!pre_fsm_command && !auto_action_code)
        {
            if (fsm_command == 0x01)
            {
                fsm_command_request->command = "auto_perlace_1";
                auto fsm_command_response = fsm_command_client->async_send_request(fsm_command_request);
            }
            else if (fsm_command == 0x02)
            {
                fsm_command_request->command = "auto_perlace_2";
                auto fsm_command_response = fsm_command_client->async_send_request(fsm_command_request);
            }
            else if (fsm_command == 0x03)
            {
                fsm_command_request->command = "auto_takeout_1";
                auto fsm_command_response = fsm_command_client->async_send_request(fsm_command_request);
            }
            else if (fsm_command == 0x04)
            {
                fsm_command_request->command = "auto_takeout_2";
                auto fsm_command_response = fsm_command_client->async_send_request(fsm_command_request);
            }
            else if (fsm_command == 0x05)
            {
                fsm_command_request->command = "little_source_land";
                auto fsm_command_response = fsm_command_client->async_send_request(fsm_command_request);
                RCLCPP_INFO(node->get_logger(), "change to little source");
            }
            else if (fsm_command == 0x07)
            {
                // RCLCPP_INFO(node->get_logger(), "get command 0x07");
                if (now_fsm_state == "big_source_land")
                {
                    fsm_command_request->command = "custom_control";
                    RCLCPP_INFO(node->get_logger(), "change to custom control");
                    auto fsm_command_response = fsm_command_client->async_send_request(fsm_command_request);
                }
                else{
                    fsm_command_request->command = "big_source_land";
                    RCLCPP_INFO(node->get_logger(), "change to big source land");
                    auto fsm_command_response = fsm_command_client->async_send_request(fsm_command_request);
                }
            }
        }
        else if (!pre_fsm_command && (auto_action_code == 0x05) && (fsm_command == 0x05))
        {
            fsm_command_request->command = "little_source_land";
            auto fsm_command_response = fsm_command_client->async_send_request(fsm_command_request);
        }

        //defend模式可以打断任何其他模式
        if (!pre_fsm_command && fsm_command == 0x08)
        {
            // RCLCPP_INFO(node->get_logger(), "get command 0x07");
            if (now_fsm_state == "defend")
            {
                fsm_command_request->command = "custom_control";
                RCLCPP_INFO(node->get_logger(), "change to custom control");
                auto fsm_command_response = fsm_command_client->async_send_request(fsm_command_request);
            }
            else{
                fsm_command_request->command = "defend";
                RCLCPP_INFO(node->get_logger(), "change to defend");
                auto fsm_command_response = fsm_command_client->async_send_request(fsm_command_request);
            }
        }

        pre_fsm_command = fsm_command;
        
        /* 发布机械臂状态 */
        robotarm_state_msg.auto_action = fsm_command;
        robotarm_state_msg.auto_action_ready = robotarm_ready;
        robotarm_state_msg.robotarm_alive = true;
        robotarm_state_publisher->publish(robotarm_state_msg);

        //通过服务读取当前的fsm状态
        fsm_state_request->get = "GET";
        fsm_state_client->async_send_request(fsm_state_request, FSMState_Callback);



        // RCLCPP_INFO(node->get_logger(), "Now State, %s",now_fsm_state.c_str());
        
        send_target_angle_to_motor();

        rclcpp::spin_some(node);
    }
    rclcpp::shutdown();
    return 0;
}