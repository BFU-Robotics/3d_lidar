#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <ModbusMaster.hpp>

#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/float32.hpp>
#include "sensor_msgs/msg/joint_state.hpp"
#include <tf2/LinearMath/Quaternion.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

class LidarDriver : public rclcpp::Node
{
  public:
    LidarDriver() : Node("lidar_driver")
    {
        // Порт для связи с микроконтролером робота
    	std::string device;
    	declare_parameter("dev", "/dev/ttyUSB0");
    	get_parameter("dev", device);

        // Настройка Modbus
        m_modbus = std::make_unique<robot::protocol::ModbusMaster>(device.c_str(), 115200);
        m_modbus->Setup();

        m_anglePub = this->create_publisher<std_msgs::msg::Float32>("lidar_angle", 10);
        m_angleTimer = this->create_wall_timer(10ms, std::bind(&LidarDriver::velocityСallback, this));

        m_jointStatesPub = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

        // Задержка для включения микроконтроллера робота 
        std::this_thread::sleep_for(std::chrono::seconds(4));
    }

    ~LidarDriver()
    {

    }

  private:
    void velocityСallback()
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        std::vector<int16_t> data = m_modbus->ReadAnalogInput(0x01, 0x0001, 1);
        if(data.size() > 0)
        {   
            float angle = data[0]*acos(-1)/180;

            // Отладочный вывод
            std::cerr << "Angle in rad " << angle << std::endl;

            //auto angleMsg = std_msgs::msg::Float32();
            //angleMsg.data = data[0];
            //m_anglePub->publish(angleMsg);

            tf2::Quaternion q;
            q.setRPY(0.0, angle, 0.0);

            auto joint_states_msg = sensor_msgs::msg::JointState();
            joint_states_msg.header.frame_id = "base_link";
            joint_states_msg.header.stamp = this->now();
            joint_states_msg.name.emplace_back("rotation_joint");
            joint_states_msg.position.push_back(angle - 1.55334);
            m_jointStatesPub->publish(joint_states_msg);

            /*// Публикация TF
            geometry_msgs::msg::TransformStamped lidar_tf;
            lidar_tf.header.frame_id = "base_link";
            lidar_tf.child_frame_id = "lidar_link";
            lidar_tf.header.stamp = this->now();
            lidar_tf.transform.translation.x = 0;
            lidar_tf.transform.translation.y = 0;
            lidar_tf.transform.translation.z = 0;
            lidar_tf.transform.rotation.x = q.x();
            lidar_tf.transform.rotation.y = q.y();
            lidar_tf.transform.rotation.z = q.z();
            lidar_tf.transform.rotation.w = q.w();
            m_tfBroadcaster->sendTransform(lidar_tf);*/
        }
    }

private:
    std::mutex m_mutex;
    std::unique_ptr<robot::protocol::ModbusMaster> m_modbus;
    rclcpp::TimerBase::SharedPtr m_angleTimer;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr m_anglePub;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_jointStatesPub;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarDriver>());
  rclcpp::shutdown();
  return 0;
}
