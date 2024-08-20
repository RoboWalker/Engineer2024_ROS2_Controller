#include "FSM/FSMState_LittleSourceLand.hpp"

using namespace std::chrono_literals;

/* 创建状态类 */
FSMState_LittleSourceLand::FSMState_LittleSourceLand():
                  FSMState(FSMStateName::LITTLE_SOURCE_LAND, "little_source_land"){}

void FSMState_LittleSourceLand::enter()
{
    /* 初始化 rosbag 读取指针 */
    rosbag2_storage::StorageOptions storage_options_1(
        {"/home/engineer/engineer_ws/src/Engineer_ROS2_controller/engineer_bag/little_resource_1_1/little_resource_1_1_0.db3",
         "sqlite3"});
    rosbag2_storage::StorageOptions storage_options_2(
        {"/home/engineer/engineer_ws/src/Engineer_ROS2_controller/engineer_bag/little_resource_2_1/little_resource_2_1_0.db3",
         "sqlite3"});
    rosbag2_storage::StorageOptions storage_options_3(
        {"/home/engineer/engineer_ws/src/Engineer_ROS2_controller/engineer_bag/little_resource_3_2/little_resource_3_2_0.db3",
         "sqlite3"});

    rosbag2_cpp::ConverterOptions converter_options(
        {rmw_get_serialization_format(),
         rmw_get_serialization_format()});

    reader_1 = std::make_unique<rosbag2_cpp::readers::SequentialReader>();
    reader_2 = std::make_unique<rosbag2_cpp::readers::SequentialReader>();
    reader_3 = std::make_unique<rosbag2_cpp::readers::SequentialReader>();
    
    reader_1->open(storage_options_1, converter_options);
    reader_2->open(storage_options_2, converter_options);
    reader_3->open(storage_options_3, converter_options);

    if (reader_1->has_next())
    {
        /* 未读取到 rosbag 末尾 */
        /* 读取 rosbag 并去序列化 */
        std::shared_ptr<rosbag2_storage::SerializedBagMessage> joint_state_bag_message = reader_1->read_next();
        auto serialized_message = rclcpp::SerializedMessage(*joint_state_bag_message->serialized_data);
        auto serializer = rclcpp::Serialization<engineer_msg::msg::JointState>();

        serializer.deserialize_message(&serialized_message, &joint_state_bag);

        this->BagPlay_Counter = 0;
        this->BagPlay_Flag = 1;
    }
    else
    {
        /* 切换到自定义控制器状态 */
        fsm.setCommand(FSMCommand::CUSTOM_CONTROL);
        return;
    }

    lift_ready = false;
    little_source_land_flag = false;
}

void FSMState_LittleSourceLand::run()
{
    engineer_msg::msg::LiftState lift_state;
    engineer_msg::msg::SucketState sucket_state;

    switch (this->BagPlay_Flag)
    {
    case 1:
        if ((this->BagPlay_Counter > 330) && (little_source_land_flag == false))
        {
            sucket_target = true;
            break;
        }

        if ((this->BagPlay_Counter < 930) && (lift_position_real < 250))
        {
            lift_state.rise = true;
            lift_state.fall = false;
        }
        else if ((this->BagPlay_Counter >= 930) && (this->BagPlay_Counter < 940) && (lift_position_real > 170))
        {
            lift_state.rise = false;
            lift_state.fall = true;
        }
        else if (robotarm_ready)
        {
            if (this->BagPlay_Counter > 970)
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
            for (int i = 0; i < ((this->BagPlay_Counter < 480) ? 2 : 4); i++)
            {
                if (reader_1->has_next())
                {
                    /* 未读取到 rosbag 末尾 */
                    /* 读取 rosbag 并去序列化 */
                    std::shared_ptr<rosbag2_storage::SerializedBagMessage> joint_state_bag_message = reader_1->read_next();
                    auto serialized_message = rclcpp::SerializedMessage(*joint_state_bag_message->serialized_data);
                    auto serializer = rclcpp::Serialization<engineer_msg::msg::JointState>();

                    serializer.deserialize_message(&serialized_message, &joint_state_bag);

                    this->BagPlay_Counter++;
                    RCLCPP_INFO(node->get_logger(), "counter: %d", this->BagPlay_Counter);
                }
                else
                {
                    /* 切换到自定义控制器状态 */
                    RCLCPP_INFO(node->get_logger(), "counter finish");

                    if (reader_2->has_next())
                    {
                        /* 未读取到 rosbag 末尾 */
                        /* 读取 rosbag 并去序列化 */
                        std::shared_ptr<rosbag2_storage::SerializedBagMessage> joint_state_bag_message = reader_2->read_next();
                        auto serialized_message = rclcpp::SerializedMessage(*joint_state_bag_message->serialized_data);
                        auto serializer = rclcpp::Serialization<engineer_msg::msg::JointState>();

                        serializer.deserialize_message(&serialized_message, &joint_state_bag);

                        this->BagPlay_Counter = 0;
                        this->BagPlay_Flag = 2;
                    }
                    else
                    {
                        /* 切换到自定义控制器状态 */
                        fsm.setCommand(FSMCommand::CUSTOM_CONTROL);
                        return;
                    }

                    lift_ready = false;
                    return;
                }
            }
        }
        sucket_target = (this->BagPlay_Counter < 935) && (this->BagPlay_Counter > 335);
        break;
    
    case 2:
        if ((this->BagPlay_Counter < 780) && (lift_position_real < 250))
        {
            lift_state.rise = true;
            lift_state.fall = false;
        }
        else if ((this->BagPlay_Counter >= 780) && (this->BagPlay_Counter < 790) && (lift_position_real > 170))
        {
            lift_state.rise = false;
            lift_state.fall = true;
        }
        else if (robotarm_ready)
        {
            if (this->BagPlay_Counter > 840)
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
            for (int i = 0; i < ((this->BagPlay_Counter < 340) ? 2 : 4); i++)
            {
                if (reader_2->has_next())
                {
                    /* 未读取到 rosbag 末尾 */
                    /* 读取 rosbag 并去序列化 */
                    std::shared_ptr<rosbag2_storage::SerializedBagMessage> joint_state_bag_message = reader_2->read_next();
                    auto serialized_message = rclcpp::SerializedMessage(*joint_state_bag_message->serialized_data);
                    auto serializer = rclcpp::Serialization<engineer_msg::msg::JointState>();

                    serializer.deserialize_message(&serialized_message, &joint_state_bag);

                    this->BagPlay_Counter++;
                    RCLCPP_INFO(node->get_logger(), "counter: %d", this->BagPlay_Counter);
                }
                else
                {
                    /* 切换到自定义控制器状态 */
                    RCLCPP_INFO(node->get_logger(), "counter finish");

                    if (reader_3->has_next())
                    {
                        /* 未读取到 rosbag 末尾 */
                        /* 读取 rosbag 并去序列化 */
                        std::shared_ptr<rosbag2_storage::SerializedBagMessage> joint_state_bag_message = reader_3->read_next();
                        auto serialized_message = rclcpp::SerializedMessage(*joint_state_bag_message->serialized_data);
                        auto serializer = rclcpp::Serialization<engineer_msg::msg::JointState>();

                        serializer.deserialize_message(&serialized_message, &joint_state_bag);

                        this->BagPlay_Counter = 0;
                        this->BagPlay_Flag = 3;
                    }
                    else
                    {
                        /* 切换到自定义控制器状态 */
                        fsm.setCommand(FSMCommand::CUSTOM_CONTROL);
                        return;
                    }

                    lift_ready = false;
                    return;
                }
            }
        }
        sucket_target = (this->BagPlay_Counter < 815);
        break;
    
    case 3:
        if (lift_position_real < 250)
        {
            lift_state.rise = true;
            lift_state.fall = false;
        }
        else if (robotarm_ready)
        {
            lift_state.rise = false;
            lift_state.fall = false;

            /* 播放 rosbag */
            for (int i = 0; i < 2; i++)
            {
                if (reader_3->has_next())
                {
                    /* 未读取到 rosbag 末尾 */
                    /* 读取 rosbag 并去序列化 */
                    std::shared_ptr<rosbag2_storage::SerializedBagMessage> joint_state_bag_message = reader_3->read_next();
                    auto serialized_message = rclcpp::SerializedMessage(*joint_state_bag_message->serialized_data);
                    auto serializer = rclcpp::Serialization<engineer_msg::msg::JointState>();

                    serializer.deserialize_message(&serialized_message, &joint_state_bag);

                    this->BagPlay_Counter++;
                    RCLCPP_INFO(node->get_logger(), "counter: %d", this->BagPlay_Counter);
                }
                else
                {
                    /* 切换到自定义控制器状态 */
                    RCLCPP_INFO(node->get_logger(), "counter finish");
                }
            }
        }
        sucket_target = true;
        break;

    default:
        fsm.setCommand(FSMCommand::CUSTOM_CONTROL);
        return;
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
    sucket_state.state = sucket_target;
    sucket_command_publisher->publish(sucket_state);
}

void FSMState_LittleSourceLand::exit()
{
    /* NULL */
}
