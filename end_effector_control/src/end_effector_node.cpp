
#include <ros/ros.h>
#include <serial/serial.h>
#include "end_effector_control/keyboard.h"

#include <iostream>
#include <thread>
#include <atomic>
#include <string>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>


std::atomic<bool> running(true);

void readArduinoSerial(serial::Serial& arduino)
{
    while (running && ros::ok())
    {
        try
        {
            if (arduino.available() > 0)
            {
                std::string line = arduino.readline(200, "\n");

                if (!line.empty())
                {
                    // Remove trailing carriage return, if present
                    while (!line.empty() &&
                           (line.back() == '\r' || line.back() == '\n'))
                    {
                        line.pop_back();
                    }

                    if (!line.empty())
                        std::cout << "[Arduino] " << line << std::endl;
                }
            }
            else
            {
                ros::Duration(0.01).sleep();
            }
        }
        catch (const std::exception& e)
        {
            ROS_ERROR_THROTTLE(2.0, "Serial read error: %s", e.what());
            ros::Duration(0.1).sleep();
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "end_effector_node");
    ros::NodeHandle nh("~");

    std::string port;
    int baudrate;

    nh.param<std::string>("port", port, "/dev/ttyARDUINO");
    nh.param("baudrate", baudrate, 115200);

    serial::Serial arduino;

    try
    {
        arduino.setPort(port);
        arduino.setBaudrate(baudrate);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(100);
        arduino.setTimeout(timeout);
        arduino.open();
    }
    catch (serial::IOException&)
    {
        ROS_ERROR("Cannot open serial port %s", port.c_str());
        return -1;
    }

    if (!arduino.isOpen())
    {
        ROS_ERROR("Serial port is not open.");
        return -1;
    }

    // Discard startup messages already in the serial buffer
    arduino.flushInput();

    ROS_INFO("Connected to Arduino on %s", port.c_str());

    ros::Subscriber command_sub =
    nh.subscribe<std_msgs::String>(
        "/end_effector/command",
        10,
        [&arduino](const std_msgs::String::ConstPtr& msg)
        {
            if (msg->data.size() == 1)
            {
                arduino.write(msg->data);
                ROS_INFO("Received automatic command: %s",
                         msg->data.c_str());
            }
        });

    // The f key starts the automatic ArUco alignment node. Latching ensures
    // the request is retained if that node is still connecting at startup.
    ros::Publisher alignment_start_pub =
        nh.advertise<std_msgs::Bool>("/deploy_auto/start", 1, true);

    Keyboard keyboard;

    std::cout << "\nEnd Effector Controller\n"
              << "m : Magnet ON\n"
              << "n : Magnet OFF\n"
              << "q : Extend + print current\n"
              << "r : Retract + print current\n"
              << "s : Stop actuator and current printing\n"
              << "a : Tracks forward\n"
              << "b : Tracks backward\n"
              << "c : Rotate CW\n"
              << "d : Rotate CCW\n"
              << "e : Stop tracks\n"
              << "f : Start automatic ArUco alignment\n"
              << "x : Quit\n" << std::endl;

    // Read Arduino output concurrently with keyboard input
    std::thread reader(readArduinoSerial, std::ref(arduino));

    ros::AsyncSpinner spinner(1);
    spinner.start();

    while (ros::ok())
    {
        char key = keyboard.getKey();

        if (key == 'x')
            break;

        if (key == 'f')
        {
            std_msgs::Bool start_msg;
            start_msg.data = true;
            alignment_start_pub.publish(start_msg);
            ROS_INFO("Requested automatic ArUco alignment (f).");
            continue;
        }

        switch (key)
        {
            case 'm': case 'n':
            case 'q': case 'r': case 's':
            case 'a': case 'b': case 'c':
            case 'd': case 'e':
                arduino.write(std::string(1, key));
                ROS_INFO("Sent command: %c", key);
                break;

            default:
                break;
        }

        //ros::spinOnce();
    }

    running = false;

    if (reader.joinable())
        reader.join();

    arduino.close();
    return 0;
}