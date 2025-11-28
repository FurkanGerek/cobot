#include "cobot_controller/cobot_controller.hpp"

namespace cobot_controller
{
    constexpr auto DEFAULT_GOAL_TOPIC = "~/goal";

    using hardware_interface::HW_IF_POSITION;

    controller_interface::CallbackReturn CobotController::on_init()
    {
        RCLCPP_INFO(get_node()->get_logger(), "on_init() fonksiyonu basladi.");

        try
        {
            param_listener_ = std::make_shared<ParamListener>(get_node());
            params_ = param_listener_->get_params();
        }
        catch (const std::exception &e)
        {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }

        joint_names_ = {
            params_.part_1_joint,
            params_.part_2_joint,
            params_.part_3_joint,
            params_.part_4_joint,
            params_.part_5_joint,
            params_.part_6_joint,
        };

        gripper_joint_names_ = {
            params_.finger_1_joint,
            params_.finger_2_joint,
        };

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration CobotController::command_interface_configuration() const
    {
        RCLCPP_INFO(get_node()->get_logger(), "Komut arayüzü konfigüre edildi.");
        std::vector<std::string> conf_names;

        for (const auto &joint_name_ : joint_names_)
        {
            conf_names.push_back(joint_name_ + "/" + HW_IF_POSITION);
        }

        for (const auto &joint_name_ : gripper_joint_names_)
        {
            conf_names.push_back(joint_name_ + "/" + HW_IF_POSITION);
        }

        return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
    }

    controller_interface::InterfaceConfiguration CobotController::state_interface_configuration() const
    {
        RCLCPP_INFO(get_node()->get_logger(), "Durum arayüzü konfigüre edildi.");

        std::vector<std::string> conf_names;

        for (const auto &joint_name_ : joint_names_)
        {
            conf_names.push_back(joint_name_ + "/" + HW_IF_POSITION);
        }

        for (const auto &joint_name_ : gripper_joint_names_)
        {
            conf_names.push_back(joint_name_ + "/" + HW_IF_POSITION);
        }

        return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
    }

    controller_interface::CallbackReturn CobotController::on_configure(const rclcpp_lifecycle::State &)
    {
        action_server_ = rclcpp_action::create_server<ForwardKinematic>(
            get_node(),
            "~/goal",
            std::bind(&CobotController::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&CobotController::handle_cancel, this, std::placeholders::_1),
            std::bind(&CobotController::handle_accepted, this, std::placeholders::_1),
            rcl_action_server_get_default_options(),
            nullptr);

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn CobotController::on_activate(const rclcpp_lifecycle::State &)
    {
        angle_ = params_.angle_degree * M_PI / 180;
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn CobotController::on_deactivate(const rclcpp_lifecycle::State &)
    {
        angle_ = 0;

        return CallbackReturn::SUCCESS;
    }

    controller_interface::return_type CobotController::update(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        // double dt = period.seconds();
        (void)time;
        (void)period;

        for (size_t i = 0; i < 6; i++)
        {
            double state = state_interfaces_[i].get_value();
            double command = state;
            double goal = goals_[i];

            if (fabs(state - goal) > angle_)
            {
                command = state - (angle_ * copysign(1, state - goal));
            }

            command_interfaces_[i].set_value(command);
        }

        return controller_interface::return_type::OK;
    }

    rclcpp_action::GoalResponse CobotController::handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const ForwardKinematic::Goal> goal)
    {
        if (goal->goal.size() != goals_.size())
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Gelen Goal dizisi boyutu (%zu) kontrolcü eklem sayısına (%zu) uymuyor!",
                         goal->goal.size(), goals_.size());
            return rclcpp_action::GoalResponse::REJECT;
        }

        std::copy(
            goal->goal.begin(),
            goal->goal.end(),
            goals_.begin());

        std::string s;
        for (double v : goals_)
        {
            s += std::to_string(v) + " ";
        }

        RCLCPP_INFO(get_node()->get_logger(), "Goal array: %s", s.c_str());
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse CobotController::handle_cancel(const std::shared_ptr<GoalHandleForwardKinematic> goal_handle)
    {
        RCLCPP_INFO(get_node()->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void CobotController::handle_accepted(const std::shared_ptr<GoalHandleForwardKinematic> goal_handle)
    {
        std::thread{std::bind(&CobotController::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void CobotController::execute(const std::shared_ptr<GoalHandleForwardKinematic> goal_handle)
    {
        RCLCPP_INFO(get_node()->get_logger(), "Executing goal");
        rclcpp::Rate loop_rate(10);

        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<ForwardKinematic::Feedback>();
        auto result = std::make_shared<ForwardKinematic::Result>();

        while (rclcpp::ok())
        {
            if (goal_handle->is_canceling())
            {
                goal_handle->canceled(result);
                RCLCPP_WARN(get_node()->get_logger(), "Goal iptal edildi.");
                return;
            }

            bool all_joints_reached = true;

            for (size_t i = 0; i < goals_.size(); i++)
            {
                double current_state = state_interfaces_[i].get_value();
                double target_goal = goals_[i];

                std::array<double, 3> feedback_forward = forward_kinematic(state_interfaces_[0].get_value(), state_interfaces_[1].get_value(), state_interfaces_[2].get_value(), state_interfaces_[3].get_value());

                feedback->x = feedback_forward[0];
                feedback->y = feedback_forward[1];
                feedback->z = feedback_forward[2];

                goal_handle->publish_feedback(feedback);
                if (fabs(current_state - target_goal) > angle_)
                {
                    all_joints_reached = false;
                    break;
                }
            }

            if (all_joints_reached)
            {
                std::array<double, 3> feedback_forward = forward_kinematic(state_interfaces_[0].get_value(), state_interfaces_[1].get_value(), state_interfaces_[2].get_value(), state_interfaces_[3].get_value());
                result->x = feedback_forward[0];
                result->y = feedback_forward[1];
                result->z = feedback_forward[2];

                goal_handle->succeed(result);
                RCLCPP_INFO(get_node()->get_logger(), "Goal başarıyla tamamlandı (Hedefe ulaşıldı).");
                return;
            }

            loop_rate.sleep();
        }

        if (!rclcpp::ok())
        {
            goal_handle->abort(result);
            RCLCPP_ERROR(get_node()->get_logger(), "Sistem kapandığı için Goal iptal edildi (Abort).");
        }
    }

    std::array<double, 3> CobotController::forward_kinematic(double Q1, double Q2, double Q3, double Q4)
    {
        double x, y, z;
        x = 0.13 * std::sin(Q1) - std::cos(Q1) * (0.446 * std::sin(Q2) + 0.361 * std::sin(Q2 + Q3) + 0.1425 * std::sin(Q2 + Q3 + Q4));
        y = -0.13 * std::cos(Q1) - std::sin(Q1) * (0.446 * std::sin(Q2) + 0.361 * std::sin(Q2 + Q3) + 0.1425 * std::sin(Q2 + Q3 + Q4));
        z = 0.446 * std::cos(Q2) + 0.361 * std::cos(Q2 + Q3) + 0.1425 * std::cos(Q2 + Q3 + Q4);
        return {x, y, z};
    }
}
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(cobot_controller::CobotController, controller_interface::ControllerInterface)
