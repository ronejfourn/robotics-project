#include <memory>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

using namespace std::chrono_literals;

class Poser {
public:
    Poser() {
        m_node = rclcpp::Node::make_shared(
            "arm_pose",
            rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
        );

        m_mgi = std::make_shared<moveit::planning_interface::MoveGroupInterface>(m_node, "arm");

        m_vacuum_node = rclcpp::Node::make_shared("vacuum_gripper");
        m_vacuum_client = m_vacuum_node->create_client<std_srvs::srv::SetBool>("switch");
    }

    void do_pickup() {
        _do("arm_pick_up_ready");
        _vac(true);
        rclcpp::sleep_for(1s);
        _do("arm_pick_up_done");
    }

    void do_drop() {
        _do("arm_drop");
        _vac(false);
        _do("arm_rest");
    }

private:
    void _do(const char *target) {
        m_mgi->setNamedTarget(target);

        auto const [success, plan] = [this] {
            moveit::planning_interface::MoveGroupInterface::Plan msg;
            auto ok = static_cast<bool>(this->m_mgi->plan(msg));
            return std::make_pair(ok, msg);
        }();

        if (success)
            m_mgi->execute(plan);
    }

    void _vac(bool val) {
        auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
        req->data = val;

        while (!m_vacuum_client->wait_for_service(1s)) {
            if (rclcpp::ok())
                return;
        }

        auto res = m_vacuum_client->async_send_request(req);
        rclcpp::spin_until_future_complete(m_vacuum_node, res);
    }

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> m_mgi;
    std::shared_ptr<rclcpp::Node> m_node;
    std::shared_ptr<rclcpp::Node> m_vacuum_node;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr m_vacuum_client;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto p = Poser();
    p.do_pickup();
    rclcpp::sleep_for(5s);
    p.do_drop();

    rclcpp::shutdown();
    return 0;
}
