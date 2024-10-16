#include "ma_radar/ma_radar_serial.hpp"
#include "ma_radar/crc.hpp"
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/UInt16.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <sstream>
#include <string>
#include <thread>
#include "ma_radar/Point2f.h"

serial::Serial ser;
std::thread receive_thread_;

using namespace std::chrono_literals;

using ReceivePacket = ma_serial_packet::ReceivePacket;
using ReceiveMianPacket = ma_serial_packet::ReceiveMianPacket;

void cmd_velCallBack(const ma_radar::Point2f &msg);

void receiveData();

void statusCallback(const actionlib_msgs::GoalStatusArrayConstPtr &status);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ma_serial_receive"); // Initialize ROS
    ros::NodeHandle nh("~");                    // Node handle

    std::string port;
    std::string topic;
    int baud_rate;
    int time_out;
    bool debug;
    cv::Point2f carm;
    carm.x = 1;
    carm.y = 1;

    if (nh.getParam("/ma_serial/port", port) && nh.getParam("/ma_serial/baud_rate", baud_rate) && nh.getParam("/ma_serial/time_out", time_out) && nh.getParam("/ma_serial/topic", topic) && nh.getParam("/ma_serial/debug", debug))
    {
        ROS_INFO("Current port name is %s, baud rate is %d.", port.c_str(), baud_rate);
    }
    else
    {
        ROS_ERROR("Failed to retrieve the port parameter.");
        return 1; // Exit with an error code
    }

    try
    {
        ser.setPort(port);
        ser.setBaudrate(baud_rate);
        serial::Timeout to = serial::Timeout::simpleTimeout(time_out);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open the serial port: " << e.what());
        return 1;
    }

    if (ser.isOpen())
    {
        ROS_INFO_STREAM("Serial port initialized.");
        receive_thread_ = std::thread(receiveData);
    }
    else
    {
        return -1;
    }

    ros::Subscriber sub = nh.subscribe("/cmd_vel", 1000, cmd_velCallBack);


    while (ros::ok())
        ros::spinOnce();

    return 0;
}

void cmd_velCallBack(const ma_radar::Point2f &msg)
{

    try
    {
        ma_serial_packet::SendPacket packet;
        packet.head = 0x5A;
        packet.data_lenth = 6;
        packet.id = 0x0305;
        packet.cpc = 0;
        packet.target_rabot_id = 101;
        packet.linear_x = msg.x;
        packet.linear_y = msg.y;

        crc16::append_crc16_checksum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));

        ROS_INFO("Current msg: v_x: %f, v_y: %f", msg.x, msg.y);

        uint8_t *packet_ptr = reinterpret_cast<uint8_t *>(&packet);

        ser.write(packet_ptr, sizeof(packet));
    }
    catch (const std::exception &ex)
    {
        ROS_INFO("Error when sending data: %s", ex.what());
    }
}

union uint16_8
{
    uint16_t u16;
    uint8_t u8[2];
};

bool readHead(uint8_t *head_buffer)
{
    std::memset(head_buffer, 0, 5);
    if (ser.read(head_buffer, 1) != 1)
    {
        printf("Read SOF Error\n");
        return false;
    }
    if (head_buffer[0] != 0xA5)
    {                                                                           
        // printf("Verify Head Error\n");
        return false;
    }

    if (ser.read(head_buffer + 1, 6) != 6)
    {
        printf("Read Head Error\n");
        return false;
    }

    bool crc_succ = crc8::Verify_CRC8_Check_Sum(head_buffer, 5);

    // printf("Head Verify %d\n", crc_succ);
    return crc_succ;
}


void receiveData()
{
    std::vector<uint8_t> header(1);
    std::vector<uint8_t> data;

    uint8_t head_buffer[5];
    uint8_t read_buffer[1025];
    data.reserve(sizeof(ReceivePacket));

    while (ros::ok())
    {

        if (readHead(read_buffer))
        {
            uint16_t id = read_buffer[6] << 8 | read_buffer[5];
            //  printf("id: %04X\n", id);

            if (id != 0x0202)
            {
                // printf("id: %04X\n", id);
            }

            if (id == 0x020C)
            {
                if((read_buffer[7]>0)&&(read_buffer[7]==120))
                {
                    ROS_INFO("mark_hero_progress success");
                }
                if((read_buffer[8]>0)&&(read_buffer[8]==120))
                {
                    ROS_INFO("mark_engineer_progress success");
                }
                if((read_buffer[9]>0)&&(read_buffer[9]==120))
                {
                    ROS_INFO("mark_standard_3_progress success");
                }
                if((read_buffer[10]>0)&&(read_buffer[10]==120))
                {
                    ROS_INFO("mark_standard_4_progress success");
                }
                if((read_buffer[11]>0)&&(read_buffer[11]==120))
                {
                    ROS_INFO("mark_standard_5_progress success");
                }
                if((read_buffer[12]>0)&&(read_buffer[12]==120))
                {
                    ROS_INFO("mark_sentry_progress success");
                }
            }
            else if (id == 0x020E)
            {
                printf("id: %04X\n", id);
                if((read_buffer[7]>0)&&(read_buffer[7]==1))
                {
                    ROS_INFO("success");
                }
            }
        }
    }
}
