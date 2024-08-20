#include "Serial.h"
#include "rclcpp/rclcpp.hpp"
#include <vector>
#include "engineer_msg/msg/end_effector.hpp"
#include "engineer_msg/msg/sucket_state.hpp"
#include "engineer_msg/msg/lift_state.hpp"
#include "CRC.h"
#include "time.h"

static WzSerialPort serial;

float xyzrpy_customctrl[6];
float xyzrpy_arm[6];

int main(){
    if (!serial.open("/dev/ttyPL2302", 115200, 0, 8, 1)) {
        RCLCPP_ERROR(rclcpp::get_logger("customctrl_serial_communication"), "cannot open serial port");
        // return -1;
    }

    rclcpp::init(0, nullptr);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("customctrl_serial_communication");

    //initialize a ROS2 publisher to publish target_end_effector
    auto target_end_effector_publisher = node->create_publisher<engineer_msg::msg::EndEffector>(
        "target_end_effector", 10);
 
    // initialize a ROS2 publisher to publish the sucket state, 0 for close, 1 for open
    auto sucket_state_publisher = node->create_publisher<engineer_msg::msg::SucketState>(
        "customctrl_sucket_state", 10);

    // initialize a ROS2 publisher to publish the lift state
    auto lift_state_publisher = node->create_publisher<engineer_msg::msg::LiftState>(
        "customctrl_lift_state", 10);

    const int rx_buf_len = 39;
    uint8_t rx_buf[rx_buf_len];
    int rx_len = 0, rx_recv_len = 0;
    int error_cnt = 0;

    while (rclcpp::ok())
    {
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
            if (!serial.open("/dev/ttyPL2302", 115200, 0, 8, 1)) {
                RCLCPP_ERROR(rclcpp::get_logger("customctrl_serial_communication"), "cannot open serial port");
            }
            continue;
        }
        

        if (rx_buf[0] != 0xA5) {
            continue;
        }

        /* 接收包头后4位 */
        rx_recv_len = 1;
        while (rx_recv_len < 5) {
            while((rx_len = serial.receive(rx_buf + rx_recv_len, 5 - rx_recv_len)) == 0);
            rx_recv_len += rx_len;
        }
        // RCLCPP_INFO(node->get_logger(), "%x %x %x %x %x", rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3], rx_buf[4]);

        //Check CRC8 for frame header, rx_buf[0] to rx_buf[3]
        uint8_t header_CRC8 = Get_CRC8_Check_Sum(rx_buf,4,0xFF);
        if (header_CRC8 != rx_buf[4]) {
            error_cnt++;
            std::cout << "header_CRC8 error, error_cnt = " << error_cnt << std::endl;
            //output checksum and received
            std::cout << "header_CRC8 = " << (int)header_CRC8 << std::endl;
            std::cout << "rx_buf[4] = " << (int)rx_buf[4] << std::endl;
            continue;
        }

        /* 接收完整包 */
        Pack_len = *(uint16_t *)&rx_buf[1] + 9;
        // RCLCPP_INFO(node->get_logger(), "%d", Pack_len);

        rx_recv_len = 5;
        while (rx_recv_len < Pack_len) {
            while((rx_len = serial.receive(rx_buf + rx_recv_len, Pack_len - rx_recv_len)) == 0);
            rx_recv_len += rx_len;
        }

        /* 判断命令位 */
        if (*(uint16_t *)&rx_buf[5] != 0x0302)
        {
            continue;
        }

        //Check CRC16 for frame data, rx_buf[0] to rx_buf[36]
        uint16_t data_CRC16 = Get_CRC16_Check_Sum(rx_buf,37,0xFFFF);
        uint16_t data_CRC16_recv = (rx_buf[38] << 8) + rx_buf[37];
        if (data_CRC16 != data_CRC16_recv) {
            error_cnt++;
            std::cout << "data_CRC16 error, error_cnt = " << error_cnt << std::endl;
            std::cout << "data_CRC16 = " << data_CRC16 << std::endl;
            std::cout << "data_CRC16_recv = " << data_CRC16_recv << std::endl;
            continue;
        }

        memcpy(xyzrpy_customctrl, rx_buf + 7, 24);
        xyzrpy_arm[0] = (xyzrpy_customctrl[0] + 0.125f) * 3.5f + 0.7f;
        xyzrpy_arm[1] = xyzrpy_customctrl[1] * 3.5f;
        xyzrpy_arm[2] = xyzrpy_customctrl[2] * 4.0;
        xyzrpy_arm[3] = xyzrpy_customctrl[3];
        xyzrpy_arm[4] = xyzrpy_customctrl[4];
        xyzrpy_arm[5] = xyzrpy_customctrl[5];
        // //output the xyzrpy
        // for (int i = 0; i < 6; i++) {
        //     std::cout << xyzrpy_arm[i] << " ";
        // }
        // std::cout << std::endl;

        //call the ROS2 end_effector_publisher to publish the end effector
        engineer_msg::msg::EndEffector end_effector;
        end_effector.position[0] = xyzrpy_arm[0];
        end_effector.position[1] = xyzrpy_arm[1];
        end_effector.position[2] = xyzrpy_arm[2];
        end_effector.orientation[0] = xyzrpy_arm[3];
        end_effector.orientation[1] = xyzrpy_arm[4];
        end_effector.orientation[2] = xyzrpy_arm[5];
        target_end_effector_publisher->publish(end_effector);

        // RCLCPP_INFO(node->get_logger(), "%f, %f, %f", xyzrpy_arm[3], xyzrpy_arm[3], xyzrpy_arm[3]);

        // 从rx_buf的第32个字节的第0位读取吸盘状态
        engineer_msg::msg::SucketState sucket_state;
        sucket_state.state = (rx_buf[31] & 0x01);
        sucket_state_publisher->publish(sucket_state);

        //从rx_buf 的第32个字节的第1,2位读取抬升状态
        engineer_msg::msg::LiftState lift_state;
        lift_state.rise = (rx_buf[31] & (0x01 << 1));
        lift_state.fall = (rx_buf[31] & (0x01 << 2));
        lift_state_publisher->publish(lift_state);

        rclcpp::spin_some(node);
    }
    rclcpp::shutdown();
    return 0;
}