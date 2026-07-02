#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/UInt32.h>
#include <vector>
#include <cstdint>
#include <cstdio>

// 定义接收缓冲区大小
const size_t RECEIVE_BUFFER_SIZE = 6; // 帧头(1) + 数据(4) + 校验和(1)

int main(int argc, char** argv)
{
    ros::init(argc, argv, "serial_receiver");
    ros::NodeHandle nh;

    std::string port;
    int baud_rate = 9600;
    nh.param("port", port, std::string("/dev/ttyS4"));  // 接收端默认串口
    nh.param("baud_rate", baud_rate, 9600);           // 默认波特率

    // 创建ROS话题发布器
    ros::Publisher data_pub = nh.advertise<std_msgs::UInt32>("received_data", 10);

    serial::Serial ser;
    try {
        ser.setPort(port);
        ser.setBaudrate(baud_rate);

        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(timeout);

        ser.open();
    } catch (serial::IOException& e) {
        ROS_ERROR_STREAM("Failed to open serial port: " << port);
        return -1;
    }

    if (!ser.isOpen()) {
        ROS_ERROR_STREAM("Serial port is not open!");
        return -1;
    }

    ROS_INFO_STREAM("Successfully opened serial port: " << port);

    std::vector<uint8_t> buffer(RECEIVE_BUFFER_SIZE);

    ros::Rate rate(10); // 每秒轮询10次
    while (ros::ok())
    {
        size_t bytes_available = ser.available();

        if (bytes_available >= RECEIVE_BUFFER_SIZE)
        {
            ser.read(&buffer[0], RECEIVE_BUFFER_SIZE);

            // 打印接收到的数据（十六进制）
            ROS_INFO_STREAM("Received (hex): ";
                for (size_t i = 0; i < RECEIVE_BUFFER_SIZE; ++i) {
                    printf("%02X ", buffer[i]);
                }
                printf("\n");
            );

            // 校验帧头
            if (buffer[0] != 0xA5) {
                ROS_WARN("Invalid header byte, expected A5");
                continue;
            }

            // 校验和计算
            uint8_t sum = 0;
            for (size_t i = 0; i < RECEIVE_BUFFER_SIZE - 1; ++i) {
                sum += buffer[i];
            }

            if (sum != buffer[RECEIVE_BUFFER_SIZE - 1]) {
                ROS_WARN("Checksum mismatch x");
                continue;
            }

            // 重组 uint32_t 数据 (大端序)
            uint32_t data = static_cast<uint32_t>(buffer[1]) << 24 |
                           static_cast<uint32_t>(buffer[2]) << 16 |
                           static_cast<uint32_t>(buffer[3]) << 8 |
                           static_cast<uint32_t>(buffer[4]);
            double Voltage_data = data * 3.3 / 10000 / 4095; // 假设数据是电压值，转换为实际值
            // 打印解析结果
            ROS_INFO_STREAM("Parsed data: 0x" << std::hex << data << " (" << std::dec << data << ")");
            ROS_INFO_STREAM("Voltage data: " << Voltage_data << " V");

            // 发布到ROS话题
            std_msgs::UInt32 msg;
            msg.data = static_cast<uint32_t>(Voltage_data * 1000); // 将电压值转换为毫伏单位发布
            data_pub.publish(msg);
            ROS_INFO_STREAM("Published voltage to topic: " << Voltage_data << " V");
        }
        else if (bytes_available > 0)
        {
            ROS_WARN_STREAM("Incomplete packet received (" << bytes_available << " bytes), waiting for full frame...");
            ser.flush(); // 清空不完整数据
        }

        ros::spinOnce();
        rate.sleep();
    }

    ser.close();
    return 0;
}