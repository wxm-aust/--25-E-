#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/Int32MultiArray.h>
// 定义全局缓冲区（6字节：帧头 + 4数据 + 校验和）
uint8_t send_buffer[6];  // 使用 uint8_t 更安全、兼容串口库
int dx = 0;
int dy = 0;
int dx_last = 0;
int dy_last = 0;
// 构造发送数据包：A5 d1 d2 d3 d4 sum
void build_send_data(uint16_t d1, uint16_t d2) {
    send_buffer[0] = 0xA5; // 帧头
    send_buffer[1] = ((d1>>8)&0xFF);   // 数据1
    send_buffer[2] = (d1&0xFF);   // 数据2
    send_buffer[3] = ((d2>>8)&0xFF);   // 数据3
    send_buffer[4] =  (d2&0xFF);   // 数据4

    int sum = 0;
    for (int i = 0; i < 5; ++i) {  // 计算前5字节的和
        sum += send_buffer[i];
    }
    send_buffer[5] = static_cast<uint8_t>(sum); // 校验和
}

// 回调函数，接收消息并解析数据
void dx_dy_Callback(const std_msgs::Int32MultiArray::ConstPtr& msg)
{
    ROS_INFO("Received array with %lu elements:", msg->data.size());

    // 打印所有元素
    // for(unsigned int i = 0; i < msg->data.size(); i++) {
    //     ROS_INFO("  data[%d] = %d", i, msg->data[i]);
    // }

    // 假设我们只关心前两个整数
    if (msg->data.size() >= 2) {
        dx = msg->data[0];
        dy = msg->data[1];
        // ROS_INFO("First integer: %d", dx);
        // ROS_INFO("Second integer: %d", dy);
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "serial_sender");
    ros::NodeHandle nh("~");

    std::string port;
    int baud_rate = 115200;
    nh.param("port", port, std::string("/dev/ttyS3"));
    nh.param("baud_rate", baud_rate, 115200);

    ros::Subscriber sub = nh.subscribe("/dx_dy", 10, dx_dy_Callback);
    serial::Serial ser;
    try {
        ser.setPort(port);
        ser.setBaudrate(baud_rate);

        // 设置超时
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

    // 模拟数据
    uint16_t data1; //roll， 蓝色舵机，对应dy
    uint16_t data2; //pitch，黑色舵机,对应dx

   

    // 每秒发送一次数据
    ros::Timer timer = nh.createTimer(ros::Duration(0.01), [&](const ros::TimerEvent&) {
        // 更新数据（可以替换为你自己的逻辑）

        data1  = dx;
        data2  = dy;
        // ROS_INFO("First integer: %d", data1);
        // ROS_INFO("Second integer: %d", data2);
        // 构建数据包
        build_send_data(data1, data2);

        // 发送数据
        ser.write(send_buffer, sizeof(send_buffer));

        // 打印发送内容（十六进制）
        // ROS_INFO_STREAM("Sent (hex): ";
        //     for (size_t i = 0; i < sizeof(send_buffer); ++i) {
        //         printf("%02X ", send_buffer[i]);
        //     }
        //     printf("\n");
        // );
        dx_last = dx;
        dy_last = dy;

    });

    ros::spin();
    ser.close();
    return 0;
}