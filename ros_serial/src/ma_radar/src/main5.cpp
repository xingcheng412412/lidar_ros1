#include <openvino/openvino.hpp>
#include <sl/Camera.hpp>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <string>
#include <iostream>
#include <ros/ros.h>
#include "ma_radar/Point2f.h"

using namespace std;
using namespace sl;

static int num;

// 置信度
float cof_threshold = 0.5;
float nms_area_threshold = 0.5;

// 模型
// std::string path = "/home/sun/ros_serial/src/ma_radar/tupian/";
std::string MODEL_PATH = "/home/sun/ros_serial/src/ma_radar/config/best_openvino_model/best.xml";
std::string DEVICE = "CPU";
cv::VideoCapture capture;

// 用于求车辆中心点
void leaf_center(cv::Mat img, std::vector<float> center_points)
{
    int x, y;
    cv::Point2f center;
    x = center_points[0];
    y = center_points[1];
    center.x = x;
    center.y = y;
    cv::circle(img, center, 5, cv::Scalar(0, 255, 0), -1);
}

// 求目标中心点
cv::Point3f object_distance(sl::Mat point_cloud, std::vector<float> center_points)
{
    int x, y;
    cv::Point2f center;
    // center_points存放着中心坐标
    x = center_points[0];
    y = center_points[1];
    center.x = x;
    center.y = y;
    sl::float4 point_cloud_value;
    point_cloud.getValue(x, y, &point_cloud_value); // point_cloud是点云图
    cv::Point3f world_point;

    if (std::isfinite(point_cloud_value.z))
    {
        float distance = sqrt(point_cloud_value.x * point_cloud_value.x + point_cloud_value.y * point_cloud_value.y + point_cloud_value.z * point_cloud_value.z) / 1000;
        cout << "Distance be " << distance << "m" << endl;
        float x1, y1, z1;
        x1 = point_cloud_value.x / 1000;
        y1 = point_cloud_value.y / 1000;
        z1 = point_cloud_value.z / 1000;
        world_point.x = x1;
        world_point.y = y1;
        world_point.z = z1;

        cout << "x: " << x1 << "; y: " << y1 << "; z: " << z1 << endl;
    }
    else
        cout << "The Distance can not be computed at {" << x << ";" << y << "}" << endl;
    return world_point;
}

// 将相机坐标转化为真实坐标
cv::Point2f convertToRealCoordinate(double cameraX, double cameraY)
{
    // 倾斜角度,需要根据比赛时的角度改
    int tiltAngle = 45;

    // 世界坐标x
    double wouldX = cameraX * cos(tiltAngle * M_PI / 180.0);

    // 世界坐标y
    double wouldY = cameraY * cos(tiltAngle * M_PI / 180.0);

    // 目标点的真实坐标
    cv::Point2f realCoord;
    realCoord.x = wouldX;
    realCoord.y = wouldY;

    return realCoord;
}

// 将我所拍下的图片防盗一个大小合适的图片上
void letterbox(const cv::Mat &source, cv::Mat &result)
{
    int col = source.cols;
    int row = source.rows;
    int _max = MAX(col, row);
    result = cv::Mat::zeros(_max, _max, CV_8UC3);
    source.copyTo(result(cv::Rect(0, 0, col, row)));
}

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "velocity_publisher");

    // 创建节点句柄
    ros::NodeHandle nh;

    // 创建发布者，发布话题为 /msg_pub/cmd_vel，消息类型为 geometry_msgs::TwistStamped
    ros::Publisher vel_pub = nh.advertise<ma_radar::Point2f>("cmd_vel", 10);

    // 设置发布频率为10Hz
    ros::Rate loop_rate(10);

    // 创建一个相机类
    Camera zed;

    // Set configuration parameters
    InitParameters init_parameters;
    init_parameters.camera_resolution = RESOLUTION::AUTO; // Use HD720 opr HD1200 video mode, depending on camera type.
    init_parameters.camera_fps = 30;                      // Set fps at 30

    // 开相机
    auto returned_state = zed.open(init_parameters);
    printf("success");
    if (returned_state != ERROR_CODE::SUCCESS)
    {
        cout << "Error " << returned_state << ", exit program." << endl;
        return EXIT_FAILURE;
    }
    // 1.Create Runtime Core
    ov::Core core;

    // 2.Compile the model
    ov::CompiledModel compiled_model = core.compile_model(MODEL_PATH, DEVICE);

    // 3.Create inference request
    ov::InferRequest infer_request = compiled_model.create_infer_request();

    float gamma = 0.5;

    // 将图像像素值应用伽马转换来降低亮度
    cv::Mat darkenedImage;
    // sl::Mat与cv::Mat是不一样的，需要转化
    sl::Mat image, depth, point_cloud;

    while (true)
    {
        // 抓取一张照片
        returned_state = zed.grab();
        // Retrieve left image
        zed.retrieveImage(image, VIEW::LEFT);
        // Retrieve depth map. Depth is aligned on the left image
        zed.retrieveMeasure(depth, MEASURE::DEPTH);
        // Retrieve colored point cloud. Point cloud is aligned on the left image.
        zed.retrieveMeasure(point_cloud, MEASURE::XYZRGBA);

        // 将拍下的图片给img1
        cv::Mat img1(image.getHeight(), image.getWidth(), CV_8UC4, image.getPtr<sl::uchar1>());
        // 将通道数改换
        cv::cvtColor(img1, img1, cv::COLOR_BGRA2BGR);

        cv::Mat letterbox_img;
        letterbox(img1, letterbox_img);
        float scale = letterbox_img.size[0] / 640.0;
        cv::Mat blob = cv::dnn::blobFromImage(letterbox_img, 1.0 / 255.0, cv::Size(640, 640), cv::Scalar(), true);

        auto input_port = compiled_model.input();

        ov::Tensor input_tensor(input_port.get_element_type(), input_port.get_shape(), blob.ptr(0));

        infer_request.set_input_tensor(input_tensor);

        infer_request.infer();

        // 获取输出
        auto output = infer_request.get_output_tensor(0);
        auto output_shape = output.get_shape();
        // std::cout<<output_shape<<std::endl;

        float *data = output.data<float>();

        cv::Mat output_buffer(output_shape[1], output_shape[2], CV_32F, data);
        transpose(output_buffer, output_buffer); //[8400,23]
        cv::Mat dst = img1.clone();
        std::vector<int> class_ids;
        int class_id;
        // num = num + 1;
        // if (!cv::imwrite(path + std::to_string(num) + ".jpg", dst))
        // {
        //     std::cerr << "Failed to save image: " << path << std::endl;
        // }
        // else
        // {
        //     std::cout << "Downloaded and saved: " << path << std::endl;
        // }
        std::cout<<output_buffer<<std::endl;

        std::vector<float> class_scores;
        std::vector<cv::Rect> boxes;
        std::vector<std::vector<float>> centers_points;

        for (int i = 0; i < output_buffer.rows; i++)
        {
            float class_score = output_buffer.at<float>(i, 4);
            if (class_score > cof_threshold)
            {
                class_scores.push_back(class_score);
                class_ids.push_back(i);
                float cx = output_buffer.at<float>(i, 0);
                float cy = output_buffer.at<float>(i, 1);
                float w = output_buffer.at<float>(i, 2);
                float h = output_buffer.at<float>(i, 3);
                // Get the box
                int left = int((cx - 0.5 * w) * scale);
                int top = int((cy - 0.5 * h) * scale);
                int width = int(w * scale);
                int height = int(h * scale);
                // Get the centerpoints
                std::vector<float> center_points;
                int center_x = left + width / 2;
                int center_y = top + height / 2;
                center_points.push_back(center_x);
                center_points.push_back(center_y);

                boxes.push_back(cv::Rect(left, top, width, height));
                centers_points.push_back(center_points);
            }
        }

        // NMS
        std::vector<int> indices;
        if (boxes.size() > 0)
        {
            cv::dnn::NMSBoxes(boxes, class_scores, cof_threshold, nms_area_threshold, indices);
        }

        // -------- Visualize the detection results -----------
        for (size_t i = 0; i < indices.size(); i++)
        {
            int index = indices[i];

            // Draw bounding box
            cv::rectangle(dst, boxes[index], cv::Scalar(0, 0, 255), 2, 8);
            std::string label = std::to_string(class_id) + ":" + std::to_string(class_scores[index]).substr(0, 4);
            cv::Size textSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, 0);
            cv::Rect textBox(boxes[index].tl().x, boxes[index].tl().y - 15, textSize.width, textSize.height + 5);
            cv::rectangle(dst, textBox, cv::Scalar(0, 0, 255), cv::FILLED);
            cv::putText(dst, label, cv::Point(boxes[index].tl().x, boxes[index].tl().y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));
            // Draw centerpoints
            std::vector<float> center_keypoints = centers_points[index];
            // 画出扇叶中心点
            leaf_center(dst, center_keypoints);
            cv::Point3f world_point = object_distance(point_cloud, center_keypoints);
            ma_radar::Point2f x;
            x.x = world_point.z;
            x.y = world_point.x;
            vel_pub.publish(x);
            ros::spinOnce();
            // loop_rate.sleep();
        }
        if (img1.empty())
            break;
        cv::imshow("dst", dst);
        // cv::waitKey(0);
        if (cv::waitKey(1) >= 0)
            break;
    }
    // Close the camera
    zed.close();
    return EXIT_SUCCESS;
}