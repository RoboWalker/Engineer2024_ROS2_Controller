#include "FSM/FSMState_AutoPerlace1.hpp"

using namespace std::chrono_literals;

/* 创建状态类 */
FSMState_AutoPerlace1::FSMState_AutoPerlace1():
                  FSMState(FSMStateName::AUTO_PERLACE_1, "auto_perlace_1"){}

/* 创建 rosbag 读取指针 */
// rosbag2_storage::StorageOptions storage_options(
//     {"/home/engineer/engineer_ws/src/Engineer_ROS2_controller/engineer_bag/auto_perlace_1_1/auto_perlace_1_1_0.db3",
//      "sqlite3"});
// rosbag2_cpp::ConverterOptions converter_options(
//     {rmw_get_serialization_format(),
//      rmw_get_serialization_format()});
// std::unique_ptr<rosbag2_cpp::readers::SequentialReader> reader_;

void FSMState_AutoPerlace1::enter()
{
    /* 初始化 rosbag 读取指针 */
    rosbag2_storage::StorageOptions storage_options(
        {"/home/engineer/engineer_ws/src/Engineer_ROS2_controller/engineer_bag/auto_perlace_1_1/auto_perlace_1_1_0.db3",
         "sqlite3"});
    rosbag2_cpp::ConverterOptions converter_options(
        {rmw_get_serialization_format(),
         rmw_get_serialization_format()});

    reader_ = std::make_unique<rosbag2_cpp::readers::SequentialReader>();
    reader_->open(storage_options, converter_options);

    if (reader_->has_next())
    {
        /* 未读取到 rosbag 末尾 */
        /* 读取 rosbag 并去序列化 */
        std::shared_ptr<rosbag2_storage::SerializedBagMessage> joint_state_bag_message = reader_->read_next();
        auto serialized_message = rclcpp::SerializedMessage(*joint_state_bag_message->serialized_data);
        auto serializer = rclcpp::Serialization<engineer_msg::msg::JointState>();

        serializer.deserialize_message(&serialized_message, &joint_state_bag);

        this->BagPlay_Counter = 0;
    }
    else
    {
        /* 切换到自定义控制器状态 */
        fsm.setCommand(FSMCommand::CUSTOM_CONTROL);
    }

    lift_ready = false;
}

void FSMState_AutoPerlace1::run()
{
    engineer_msg::msg::LiftState lift_state;
    engineer_msg::msg::SucketState sucket_state;

    if ((this->BagPlay_Counter < 410) && (lift_position_real < 250))
    {
        lift_state.rise = true;
        lift_state.fall = false;
    }
    else if ((this->BagPlay_Counter >= 410) && (this->BagPlay_Counter < 420) && (lift_position_real > 140))
    {
        lift_state.rise = false;
        lift_state.fall = true;
    }
    else if (robotarm_ready)
    {
        if (this->BagPlay_Counter > 450)
        {
            lift_state.rise = true;
            lift_state.fall = false;
        }
        else
        {
            lift_state.rise = false;
            lift_state.fall = false;
        }
        

        /* 播放 rosbag */
        for (int i = 0; i < 4; i++)
        {
            if (reader_->has_next())
            {
                /* 未读取到 rosbag 末尾 */
                /* 读取 rosbag 并去序列化 */
                std::shared_ptr<rosbag2_storage::SerializedBagMessage> joint_state_bag_message = reader_->read_next();
                auto serialized_message = rclcpp::SerializedMessage(*joint_state_bag_message->serialized_data);
                auto serializer = rclcpp::Serialization<engineer_msg::msg::JointState>();

                serializer.deserialize_message(&serialized_message, &joint_state_bag);

                this->BagPlay_Counter++;
                RCLCPP_INFO(node->get_logger(), "counter: %d", this->BagPlay_Counter);
            }
            else
            {
                /* 切换到自定义控制器状态 */
                robotarm_state_code = 0x00;
                RCLCPP_INFO(node->get_logger(), "counter finish");
                fsm.setCommand(FSMCommand::CUSTOM_CONTROL);
                return;
            }
        }
    }



    /* 写入并发布话题 jpoint_command */
    engineer_msg::msg::JointCommand joint_command;
    joint_command.position[0] = joint_state_bag.position[0];
    joint_command.position[1] = joint_state_bag.position[1];
    joint_command.position[2] = joint_state_bag.position[2];
    joint_command.position[3] = joint_state_bag.position[3];
    joint_command.position[4] = joint_state_bag.position[4];
    joint_command.position[5] = remain_end_roll;
    joint_command_publisher->publish(joint_command);

    // RCLCPP_INFO(node->get_logger(),"%f , %f , %f, %f , %f , %f", 
    //             joint_command.position[0], joint_command.position[1], joint_command.position[2],
    //             joint_command.position[3], joint_command.position[4], joint_command.position[5]);

    lift_command_publisher->publish(lift_state);

    /* 写入并发布话题 sucket_command */
    sucket_target = (this->BagPlay_Counter < 415);
    sucket_state.state = sucket_target;
    sucket_command_publisher->publish(sucket_state);
}

void FSMState_AutoPerlace1::exit()
{
    /* NULL */
}
