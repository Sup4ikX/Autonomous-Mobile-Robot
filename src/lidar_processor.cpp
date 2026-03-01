#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <memory>
#include <optional>
#include <filesystem>
#include <cmath>
#include <vector>
#include <chrono>
#include <cstdint>

#include <libserial/SerialPort.h>

LibSerial::BaudRate toBaudRate(uint32_t baud) {
    using namespace LibSerial;
    switch (baud) {
        case 9600:   return BaudRate::BAUD_9600;
        case 19200:  return BaudRate::BAUD_19200;
        case 38400:  return BaudRate::BAUD_38400;
        case 57600:  return BaudRate::BAUD_57600;
        case 115200: return BaudRate::BAUD_115200;
        case 230400: return BaudRate::BAUD_230400;
        case 460800: return BaudRate::BAUD_460800;
        case 921600: return BaudRate::BAUD_921600;
        default:     return BaudRate::BAUD_115200;
    }
}

class LidarProcessor : public rclcpp::Node {
private:
    std::unique_ptr<LibSerial::SerialPort> serial_port;
    
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub;
    
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    
    std::string serial_port_name;
    uint32_t serial_baud_rate;
    std::string frame_id;
    std::string base_frame_id;
    double target_rpm;
    double range_min;
    double range_max;
    
    int packet_count = 0;
    int error_count = 0;
    std::vector<double> last_scan;
    
    rclcpp::TimerBase::SharedPtr process_timer;

public:
    LidarProcessor() : rclcpp::Node("lidar_processor") {
        this->declare_parameter("serial_port", "/dev/ttyUSB0");
        this->declare_parameter("serial_baud", 115200);
        this->declare_parameter("frame_id", "laser");
        this->declare_parameter("base_frame_id", "base_link");
        this->declare_parameter("target_rpm", 300.0);
        this->declare_parameter("range_min", 0.15);
        this->declare_parameter("range_max", 12.0);
        
        serial_port_name = this->get_parameter("serial_port").as_string();
        serial_baud_rate = this->get_parameter("serial_baud").as_int();
        frame_id = this->get_parameter("frame_id").as_string();
        base_frame_id = this->get_parameter("base_frame_id").as_string();
        target_rpm = this->get_parameter("target_rpm").as_double();
        range_min = this->get_parameter("range_min").as_double();
        range_max = this->get_parameter("range_max").as_double();
        
        scan_pub = this->create_publisher<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS());
        
        status_pub = this->create_publisher<std_msgs::msg::String>(
            "/lidar_status", 10);
        
        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);
        
        try {
            auto port = std::make_unique<LibSerial::SerialPort>();
            
            // Убираем SetTimeout, таймаут будем передавать при чтении
            
            port->Open(serial_port_name);
            
            port->SetBaudRate(toBaudRate(serial_baud_rate));
            port->SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
            port->SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
            port->SetParity(LibSerial::Parity::PARITY_NONE);
            port->SetStopBits(LibSerial::StopBits::STOP_BITS_1);
            
            serial_port = std::move(port);
            
            if (serial_port->IsOpen()) {
                RCLCPP_INFO(this->get_logger(), "✓ Серийный порт открыт: %s", 
                    serial_port_name.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "✗ Не удалось открыть порт: %s", 
                    serial_port_name.c_str());
                return;
            }
        } catch (const LibSerial::OpenFailed& e) {
            RCLCPP_ERROR(this->get_logger(), "✗ Ошибка открытия порта: %s", e.what());
            return;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "✗ Ошибка порта: %s", e.what());
            return;
        }
        
        process_timer = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&LidarProcessor::process_lidar_data, this));
        
        RCLCPP_INFO(this->get_logger(), "✓ LiDAR Processor инициализирован");
    }

    void process_lidar_data() {
        if (!serial_port || !serial_port->IsOpen()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "⚠️ Серийный порт не открыт");
            return;
        }
        
        try {
            static std::vector<uint8_t> packet_buffer(256);
            static bool packet_started = false;
            static int buffer_idx = 0;
            static int packet_length = 0;

            while (serial_port->GetNumberOfBytesAvailable() > 0) {
                char byte_char;
                serial_port->ReadByte(byte_char, 1000); // таймаут 1000 мс
                uint8_t byte = static_cast<uint8_t>(byte_char);
                
                if (!packet_started) {
                    if (byte == 0xAA) {
                        packet_started = true;
                        buffer_idx = 0;
                        packet_buffer[buffer_idx++] = byte;
                    }
                    continue;
                }

                if (buffer_idx < static_cast<int>(packet_buffer.size())) {
                    packet_buffer[buffer_idx++] = byte;
                }

                if (buffer_idx >= 9) {
                    if (packet_length == 0) {
                        packet_length = (packet_buffer[2] << 8) | packet_buffer[1];
                        if (packet_length > static_cast<int>(packet_buffer.size())) {
                            packet_started = false;
                            buffer_idx = 0;
                            packet_length = 0;
                            continue;
                        }
                    }

                    if (buffer_idx >= packet_length) {
                        processLidarPacket(packet_buffer.data(), buffer_idx);
                        
                        packet_started = false;
                        buffer_idx = 0;
                        packet_length = 0;
                    }
                }
            }
            
        } catch (const LibSerial::ReadTimeout& e) {
            // Нормально, просто нет данных
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "✗ Ошибка обработки: %s", e.what());
            error_count++;
        }
    }

    void processLidarPacket(uint8_t* packet, int length) {
        if (packet[0] != 0xAA) {
            error_count++;
            return;
        }

        uint16_t received_checksum = (packet[length-1] << 8) | packet[length-2];
        uint16_t calculated_checksum = 0;
        
        for (int i = 0; i < length - 2; i++) {
            calculated_checksum += packet[i];
        }
        
        if (received_checksum != calculated_checksum) {
            error_count++;
            return;
        }

        // Эти переменные пока не используются, но могут понадобиться для диагностики
        // Можно закомментировать, чтобы избежать предупреждений
        // uint16_t packet_len = (packet[2] << 8) | packet[1];
        // uint8_t protocol_version = packet[3];
        // uint8_t packet_type = packet[4];
        uint16_t data_len = (packet[7] << 8) | packet[6];
        // uint8_t motor_speed = packet[8];
        // int16_t zero_offset = (packet[10] << 8) | packet[9];
        uint16_t start_angle = (packet[12] << 8) | packet[11];

        int data_start = 13;
        int num_samples = data_len / 3;

        std::vector<double> ranges(360, std::numeric_limits<double>::infinity());
        std::vector<double> intensities(360, 0.0);

        for (int i = 0; i < num_samples; i++) {
            int sample_idx = data_start + i * 3;
            
            if (sample_idx + 2 >= length) break;
            
            uint8_t signal_level = packet[sample_idx];
            uint16_t distance_raw = (packet[sample_idx + 2] << 8) | packet[sample_idx + 1];
            double distance_m = distance_raw * 0.25 / 1000.0;
            double angle_deg = start_angle * 0.01 + i * (data_len / num_samples) * 0.01;
            
            int angle_index = static_cast<int>(angle_deg);
            if (angle_index >= 0 && angle_index < 360) {
                if (distance_m >= range_min && distance_m <= range_max) {
                    ranges[angle_index] = distance_m;
                    intensities[angle_index] = signal_level / 255.0;
                    packet_count++;
                }
            }
        }
        
        publish_scan(ranges, intensities);
    }

    void publish_scan(const std::vector<double>& ranges, 
                     const std::vector<double>& intensities) {
        auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
        
        scan_msg->header.stamp = this->get_clock()->now();
        scan_msg->header.frame_id = frame_id;
        
        scan_msg->angle_min = -M_PI;
        scan_msg->angle_max = M_PI;
        scan_msg->angle_increment = (scan_msg->angle_max - scan_msg->angle_min) / ranges.size();
        scan_msg->time_increment = 0.0;
        scan_msg->scan_time = 1.0 / (target_rpm / 60.0);
        scan_msg->range_min = range_min;
        scan_msg->range_max = range_max;
        
        // Преобразование из double в float
        scan_msg->ranges.assign(ranges.begin(), ranges.end());
        scan_msg->intensities.assign(intensities.begin(), intensities.end());
        
        scan_pub->publish(std::move(scan_msg));
        
        publish_tf();
    }

    void publish_tf() {
        auto transform = geometry_msgs::msg::TransformStamped();
        
        transform.header.stamp = this->get_clock()->now();
        transform.header.frame_id = base_frame_id;
        transform.child_frame_id = frame_id;
        
        transform.transform.translation.x = 0.1;
        transform.transform.translation.y = 0.0;
        transform.transform.translation.z = 0.05;
        
        transform.transform.rotation.x = 0.0;
        transform.transform.rotation.y = 0.0;
        transform.transform.rotation.z = 0.0;
        transform.transform.rotation.w = 1.0;
        
        tf_broadcaster->sendTransform(transform);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<LidarProcessor>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        std::cerr << "Fatal error: " << e.what() << std::endl;
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}