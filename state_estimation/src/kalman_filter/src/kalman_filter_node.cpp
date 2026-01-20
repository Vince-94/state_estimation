/*
    ControllerNode
    --------------
    Publishers:
    - /true_odom                [nav_msgs/msg/Odometry]
    - /estimated_odom           [nav_msgs/msg/Odometry]
    - /true_path                [nav_msgs/msg/Path]
    - /estimated_path           [nav_msgs/msg/Path]
    - /sensor_marker            [visualization_msgs/msg/Marker]

    Parameters:
    - pub_freq                  [int]
    - Ts                        [double]
    - x0                        [array(double)]
    - u0                        [array(double)]
    - P0                        [array(double)]
    - Q                         [array(double)]
    - R                         [array(double)]
*/
#include "kalman_filter/linear_kalman_filter.hpp"

// ROS2 libraries
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

// ROS2 messages
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"


using namespace std::chrono_literals;


struct ConstVel1dModel {

    double dt;

    Eigen::MatrixXd A{
        {1.0, dt},
        {0.0, 1.0}
    };
    Eigen::MatrixXd B{
        {dt*dt/2.0},
        {dt}
    };
    Eigen::MatrixXd C{
        {1.0, 0.0}
    };

    explicit ConstVel1dModel() = default;

    explicit ConstVel1dModel(double dt_) : dt(dt_) {}

    Eigen::VectorXd updateState(Eigen::VectorXd x0, Eigen::VectorXd u) {
        // position += velocity * dt;
        auto pos = x0(0) + x0(1) * dt + u(0) * dt * dt;
        auto vel = x0(1);
        Eigen::VectorXd x{
            {pos, vel}
        };
        return x;
    }
};


class ControllerNode : public rclcpp::Node {
  public:
    ControllerNode(std::string name) : Node(name) {
        RCLCPP_INFO(this->get_logger(), "Node: %s", this->get_name());

        // ───────────────────────────────────────────────
        // Parameters
        // ───────────────────────────────────────────────
        // pub_freq
        declare_parameter<int>("pub_freq", pub_freq_);
        pub_freq_param = get_parameter("pub_freq");
        int pub_freq_val = static_cast<int>(pub_freq_param.as_int());

        // Ts
        declare_parameter<double>("Ts");
        Ts_param = get_parameter("Ts");
        Ts_ = Ts_param.as_double();

        // x0
        declare_parameter<std::vector<double>>("x0");
        x0_param = get_parameter("x0");
        std::vector<double> x0_val = x0_param.as_double_array();
        Eigen::VectorXd x0 = Eigen::Map<Eigen::VectorXd>(x0_val.data(), static_cast<Eigen::Index>(x0_val.size()));

        // u0
        declare_parameter<std::vector<double>>("u0");
        u0_param = get_parameter("u0");
        std::vector<double> u0_val = u0_param.as_double_array();
        Eigen::VectorXd u0 = Eigen::Map<Eigen::VectorXd>(u0_val.data(), static_cast<Eigen::Index>(u0_val.size()));

        // P0
        declare_parameter<std::vector<double>>("P0");
        P0_param = get_parameter("P0");
        std::vector<double> P0_val = P0_param.as_double_array();
        if (static_cast<int>(P0_val.size()) != x0.size()) {
            throw std::invalid_argument(
                "P0 dimensions are not consistent: " + std::to_string(P0_val.size()) + " != " + std::to_string(x0.size())
            );
        }
        Eigen::MatrixXd P0 = Eigen::MatrixXd::Zero(P0_val.size(), P0_val.size());
        P0.diagonal() = Eigen::VectorXd::Map(P0_val.data(), P0_val.size());

        // Q
        declare_parameter<std::vector<double>>("Q");
        Q_param = get_parameter("Q");
        std::vector<double> Q_val = Q_param.as_double_array();
        if (static_cast<int>(Q_val.size()) != x0.size()) {
            throw std::invalid_argument(
                "Q dimensions are not consistent: " + std::to_string(Q_val.size()) + " != " + std::to_string(x0.size())
            );
        }
        Q = Eigen::MatrixXd::Zero(Q_val.size(), Q_val.size());
        Q.diagonal() = Eigen::VectorXd::Map(Q_val.data(), Q_val.size());

        // R
        declare_parameter<std::vector<double>>("R");
        R_param = get_parameter("R");
        std::vector<double> R_val = R_param.as_double_array();
        if (static_cast<int>(R_val.size()) != u0.size()) {
            throw std::invalid_argument(
                "Q dimensions are not consistent: " + std::to_string(R_val.size()) + " != " + std::to_string(u0.size())
            );
        }
        R = Eigen::MatrixXd::Zero(R_val.size(), R_val.size());
        R.diagonal() = Eigen::VectorXd::Map(R_val.data(), R_val.size());


        RCLCPP_INFO(get_logger(), "Parameters:");
        RCLCPP_INFO(get_logger(), "* %s = %s Hz", pub_freq_param.get_name().c_str(), pub_freq_param.value_to_string().c_str());
        RCLCPP_INFO(get_logger(), "* %s = %s s", Ts_param.get_name().c_str(), Ts_param.value_to_string().c_str());
        RCLCPP_INFO(get_logger(), "* %s = %s", x0_param.get_name().c_str(), x0_param.value_to_string().c_str());
        RCLCPP_INFO(get_logger(), "* %s = %s", u0_param.get_name().c_str(), u0_param.value_to_string().c_str());
        RCLCPP_INFO(get_logger(), "* %s = %s", P0_param.get_name().c_str(), P0_param.value_to_string().c_str());
        RCLCPP_INFO(get_logger(), "* %s = %s", Q_param.get_name().c_str(), Q_param.value_to_string().c_str());
        RCLCPP_INFO(get_logger(), "* %s = %s", R_param.get_name().c_str(), R_param.value_to_string().c_str());


        // ───────────────────────────────────────────────
        // Kalman Filter
        // ───────────────────────────────────────────────
        // 1. Create system model
        const_vel_1d_sys = ConstVel1dModel(Ts_);
        StateSpace sys_model(const_vel_1d_sys.A, const_vel_1d_sys.B, const_vel_1d_sys.C);

        // 2. Create filter
        lkf = std::make_shared<LinearKalmanFilter>(std::move(sys_model));

        // 3. Initialize
        x = x0;
        u = u0;
        P = P0;

        lkf->initialize(x, P);

        // ───────────────────────────────────────────────
        // ROS2 interface
        // ───────────────────────────────────────────────

        // Publishers and TF
        true_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/true_odom", 10);
        estimated_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/estimated_odom", 10);
        true_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/true_path", 10);
        estimated_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/estimated_path", 10);
        sensor_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/sensor_marker", 10);

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Initialize paths
        true_path_.header.frame_id = "map";
        estimated_path_.header.frame_id = "map";

        // Timers
        float period_ms = (1.0f / pub_freq_val) * 1000.0f;
        std::chrono::duration<float, std::milli> pub_period{period_ms};
        pub_timer = create_wall_timer(pub_period, std::bind(&ControllerNode::timer_callback, this));
    }


  protected:
    void timer_callback() {
        // Update model state
        x = const_vel_1d_sys.updateState(x, u);

        // Simulate noisy measurement
        double meas_noise = 0.5 * (static_cast<double>(rand()) / RAND_MAX - 0.5) * 2.0;

        Eigen::VectorXd z(1);
        z(0) = x(0) + meas_noise;

        // Kalman filter steps
        lkf->predict(u, Q);
        lkf->update(z, R);

        // Get estimate
        Eigen::VectorXd x_est = lkf->getStatePost();
        Eigen::MatrixXd P_est = lkf->getCovariancePost();

        // ───────────────────────────────────────────────
        // Publish Visualizations
        // ───────────────────────────────────────────────

        auto stamp = this->now();

        // Helper: Convert 1D position/velocity to 3D pose/twist (Y=Z=0, no orientation)
        geometry_msgs::msg::Pose pose;
        pose.position.x = x(0);  // True position
        pose.position.y = 0.0;
        pose.position.z = 0.0;
        pose.orientation.w = 1.0;  // Identity quaternion

        geometry_msgs::msg::Twist twist;
        twist.linear.x = x(1);  // True velocity
        twist.linear.y = 0.0;
        twist.linear.z = 0.0;

        // True Odometry
        nav_msgs::msg::Odometry true_odom;
        true_odom.header.stamp = stamp;
        true_odom.header.frame_id = "map";
        true_odom.child_frame_id = "true_base_link";
        true_odom.pose.pose = pose;
        true_odom.twist.twist = twist;
        true_odom_pub_->publish(true_odom);

        // Estimated Odometry (with covariance)
        geometry_msgs::msg::Pose est_pose = pose;  // Reuse and override
        est_pose.position.x = x_est(0);
        geometry_msgs::msg::Twist est_twist = twist;
        est_twist.linear.x = x_est(1);

        nav_msgs::msg::Odometry est_odom;
        est_odom.header.stamp = stamp;
        est_odom.header.frame_id = "map";
        est_odom.child_frame_id = "base_link";
        est_odom.pose.pose = est_pose;
        est_odom.twist.twist = est_twist;
        // ── Pose covariance (for ellipsoid) ──────────────────────────────
        std::fill(est_odom.pose.covariance.begin(), est_odom.pose.covariance.end(), 0.0);
        est_odom.pose.covariance[0]  = P_est(0,0);     // var(position.x)
        est_odom.pose.covariance[7]  = 0.01;           // var(position.y) — small or 0
        est_odom.pose.covariance[14] = 0.0;            // var(position.z)
        est_odom.pose.covariance[21] = 0.0;            // var(roll)
        est_odom.pose.covariance[28] = 0.0;            // var(pitch)
        est_odom.pose.covariance[35] = 0.01;           // var(yaw) — give some value
        // ── Twist covariance (for velocity arrows if enabled) ─────────────
        std::fill(est_odom.twist.covariance.begin(), est_odom.twist.covariance.end(), 0.0);
        est_odom.twist.covariance[0]  = P_est(1,1);    // var(velocity.linear.x)
        est_odom.twist.covariance[7]  = 0.0;           // var(vy)
        est_odom.twist.covariance[14] = 0.0;           // var(vz)
        estimated_odom_pub_->publish(est_odom);

        // Broadcast TF (for estimated to map)
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = stamp;
        tf.header.frame_id = "map";
        tf.child_frame_id = "base_link";
        tf.transform.translation.x = x_est(0);
        tf.transform.translation.y = 0.0;
        tf.transform.translation.z = 0.0;
        tf.transform.rotation.w = 1.0;
        tf_broadcaster_->sendTransform(tf);

        // Accumulate and Publish Paths (every step or throttle to every 10 steps)
        geometry_msgs::msg::PoseStamped true_pose_stamped;
        true_pose_stamped.header.stamp = stamp;
        true_pose_stamped.header.frame_id = "map";
        true_pose_stamped.pose = pose;
        true_path_.poses.push_back(true_pose_stamped);
        true_path_.header.stamp = stamp;
        true_path_pub_->publish(true_path_);

        geometry_msgs::msg::PoseStamped est_pose_stamped;
        est_pose_stamped.header.stamp = stamp;
        est_pose_stamped.header.frame_id = "map";
        est_pose_stamped.pose = est_pose;
        estimated_path_.poses.push_back(est_pose_stamped);
        estimated_path_.header.stamp = stamp;
        estimated_path_pub_->publish(estimated_path_);

        // Sensor Marker (e.g., simple cone frustum at measured position z(0))
        visualization_msgs::msg::Marker sensor_marker;
        sensor_marker.header.frame_id = "map";
        sensor_marker.header.stamp = stamp;
        sensor_marker.ns = "sensor_frustum";
        sensor_marker.id = 0;
        sensor_marker.type = visualization_msgs::msg::Marker::SPHERE;
        sensor_marker.action = visualization_msgs::msg::Marker::ADD;
        sensor_marker.pose.position.x = z(0);  // Measured position
        sensor_marker.pose.position.y = 0.0;
        sensor_marker.pose.position.z = 0.0;
        sensor_marker.pose.orientation.w = 1.0;
        sensor_marker.scale.x = 0.5;              // adjust size to represent uncertainty scale
        sensor_marker.scale.y = 0.5;
        sensor_marker.scale.z = 0.5;
        sensor_marker.color.a = 0.6;              // semi-transparent
        sensor_marker.color.r = 1.0;
        sensor_marker.color.g = 0.4;
        sensor_marker.color.b = 0.0;
        sensor_marker.lifetime = rclcpp::Duration::from_seconds(0);  // persistent (or set short lifetime to fade old ones)
        sensor_marker_pub_->publish(sensor_marker);
    }


  private:
    int pub_freq_ = 10;
    double Ts_ = 0.1;

    std::shared_ptr<LinearKalmanFilter> lkf{};
    ConstVel1dModel const_vel_1d_sys{};

    Eigen::VectorXd x;
    Eigen::MatrixXd P;
    Eigen::VectorXd u;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;

    // Params
    rclcpp::Parameter pub_freq_param{};
    rclcpp::Parameter Ts_param{};
    rclcpp::Parameter x0_param{};
    rclcpp::Parameter u0_param{};
    rclcpp::Parameter P0_param{};
    rclcpp::Parameter Q_param{};
    rclcpp::Parameter R_param{};

    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr true_odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr estimated_odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr true_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr estimated_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr sensor_marker_pub_;

    // TF Broadcaster
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Paths
    nav_msgs::msg::Path true_path_;
    nav_msgs::msg::Path estimated_path_;

    // Timers
    rclcpp::TimerBase::SharedPtr pub_timer{};
};


int main(int argc, char** argv) {
    std::string node_name = "kalman_filter_node";

    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<ControllerNode>(node_name));

    rclcpp::shutdown();

    return 0;
}