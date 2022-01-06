//''' 
//    Purpose: Classification and BBox Regression - ROSified
//    Note: Topic names can be modified in yolo_console file
//    Subscribed topics: /pylon_camera_node_center/image_rect
//    Published topic: /bbox_array
//
//    Project: WATonoBus
//    Author: Neel Bhatt    
//
//    DO NOT share without permission from the author
//'''

#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <queue>
#include <fstream>
#include <thread>
#include <future>
#include <atomic>
#include <mutex>         // std::mutex, std::unique_lock
#include <cmath>


// It makes sense only for video-Camera (not for video-File)
// To use - uncomment the following line. Optical-flow is supported only by OpenCV 3.x - 4.x
//#define TRACK_OPTFLOW
#define GPU 1

#define isStreamCompressed 0

// To use 3D-stereo camera ZED - uncomment the following line. ZED_SDK should be installed.
//#define ZED_STEREO

#define BASLER_CAMERA 1

#include "yolo_v2_class.hpp"    // imported functions from DLL

#ifdef OPENCV
#ifdef ZED_STEREO
#include <sl/Camera.hpp>
#if ZED_SDK_MAJOR_VERSION == 2
#define ZED_STEREO_2_COMPAT_MODE
#endif

#undef GPU // avoid conflict with sl::MEM::GPU

#ifdef ZED_STEREO_2_COMPAT_MODE
#pragma comment(lib, "sl_core64.lib")
#pragma comment(lib, "sl_input64.lib")
#endif
#pragma comment(lib, "sl_zed64.lib")

float getMedian(std::vector<float> &v) {
    size_t n = v.size() / 2;
    std::nth_element(v.begin(), v.begin() + n, v.end());
    return v[n];
}

std::vector<bbox_t> get_3d_coordinates(std::vector<bbox_t> bbox_vect, cv::Mat xyzrgba)
{
    bool valid_measure;
    int i, j;
    const unsigned int R_max_global = 10;

    std::vector<bbox_t> bbox3d_vect;

    for (auto &cur_box : bbox_vect) {

        const unsigned int obj_size = std::min(cur_box.w, cur_box.h);
        const unsigned int R_max = std::min(R_max_global, obj_size / 2);
        int center_i = cur_box.x + cur_box.w * 0.5f, center_j = cur_box.y + cur_box.h * 0.5f;

        std::vector<float> x_vect, y_vect, z_vect;
        for (int R = 0; R < R_max; R++) {
            for (int y = -R; y <= R; y++) {
                for (int x = -R; x <= R; x++) {
                    i = center_i + x;
                    j = center_j + y;
                    sl::float4 out(NAN, NAN, NAN, NAN);
                    if (i >= 0 && i < xyzrgba.cols && j >= 0 && j < xyzrgba.rows) {
                        cv::Vec4f &elem = xyzrgba.at<cv::Vec4f>(j, i);  // x,y,z,w
                        out.x = elem[0];
                        out.y = elem[1];
                        out.z = elem[2];
                        out.w = elem[3];
                    }
                    valid_measure = std::isfinite(out.z);
                    if (valid_measure)
                    {
                        x_vect.push_back(out.x);
                        y_vect.push_back(out.y);
                        z_vect.push_back(out.z);
                    }
                }
            }
        }

        if (x_vect.size() * y_vect.size() * z_vect.size() > 0)
        {
            cur_box.x_3d = getMedian(x_vect);
            cur_box.y_3d = getMedian(y_vect);
            cur_box.z_3d = getMedian(z_vect);
        }
        else {
            cur_box.x_3d = NAN;
            cur_box.y_3d = NAN;
            cur_box.z_3d = NAN;
        }

        bbox3d_vect.emplace_back(cur_box);
    }

    return bbox3d_vect;
}

cv::Mat slMat2cvMat(sl::Mat &input) {
    int cv_type = -1; // Mapping between MAT_TYPE and CV_TYPE
    if(input.getDataType() ==
#ifdef ZED_STEREO_2_COMPAT_MODE
        sl::MAT_TYPE_32F_C4
#else
        sl::MAT_TYPE::F32_C4
#endif
        ) {
        cv_type = CV_32FC4;
    } else cv_type = CV_8UC4; // sl::Mat used are either RGBA images or XYZ (4C) point clouds
    return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(
#ifdef ZED_STEREO_2_COMPAT_MODE
        sl::MEM::MEM_CPU
#else
        sl::MEM::CPU
#endif
        ));
}

cv::Mat zed_capture_rgb(sl::Camera &zed) {
    sl::Mat left;
    zed.retrieveImage(left);
    cv::Mat left_rgb;
    cv::cvtColor(slMat2cvMat(left), left_rgb, CV_RGBA2RGB);
    return left_rgb;
}

cv::Mat zed_capture_3d(sl::Camera &zed) {
    sl::Mat cur_cloud;
    zed.retrieveMeasure(cur_cloud,
#ifdef ZED_STEREO_2_COMPAT_MODE
        sl::MEASURE_XYZ
#else
        sl::MEASURE::XYZ
#endif
        );
    return slMat2cvMat(cur_cloud).clone();
}

static sl::Camera zed; // ZED-camera

#else   // ZED_STEREO
std::vector<bbox_t> get_3d_coordinates(std::vector<bbox_t> bbox_vect, cv::Mat xyzrgba) {
    return bbox_vect;
}
#endif  // ZED_STEREO

// ADDED BY NEEL
// Include files to use the pylon API.
#ifdef BASLER_CAMERA
    // ROS Includes
    #include <ros/ros.h>
    #include <ros/package.h>
    #include <sensor_msgs/Image.h>
    #include <jsk_recognition_msgs/BoundingBox.h>
    #include <jsk_recognition_msgs/BoundingBoxArray.h>
    #include <cv_bridge/cv_bridge.h>
    #include <image_transport/image_transport.h>
#ifdef PYLON_WIN_BUILD
#include <pylon/PylonGUI.h>
#endif
// Namespace for using OpenCV objects.
using namespace cv;
// Namespace for using cout.
using namespace std;
// Mat temp_openCvImage;
Mat openCvImage;
#endif


#include <opencv2/opencv.hpp>            // C++
#include <opencv2/core/version.hpp>
#ifndef CV_VERSION_EPOCH     // OpenCV 3.x and 4.x
#include <opencv2/videoio/videoio.hpp>
#define OPENCV_VERSION CVAUX_STR(CV_VERSION_MAJOR)"" CVAUX_STR(CV_VERSION_MINOR)"" CVAUX_STR(CV_VERSION_REVISION)
#ifndef USE_CMAKE_LIBS
#pragma comment(lib, "opencv_world" OPENCV_VERSION ".lib")
#ifdef TRACK_OPTFLOW
/*
#pragma comment(lib, "opencv_cudaoptflow" OPENCV_VERSION ".lib")
#pragma comment(lib, "opencv_cudaimgproc" OPENCV_VERSION ".lib")
#pragma comment(lib, "opencv_core" OPENCV_VERSION ".lib")
#pragma comment(lib, "opencv_imgproc" OPENCV_VERSION ".lib")
#pragma comment(lib, "opencv_highgui" OPENCV_VERSION ".lib")
*/
#endif    // TRACK_OPTFLOW
#endif    // USE_CMAKE_LIBS
#else     // OpenCV 2.x
#define OPENCV_VERSION CVAUX_STR(CV_VERSION_EPOCH)"" CVAUX_STR(CV_VERSION_MAJOR)"" CVAUX_STR(CV_VERSION_MINOR)
#ifndef USE_CMAKE_LIBS
#pragma comment(lib, "opencv_core" OPENCV_VERSION ".lib")
#pragma comment(lib, "opencv_imgproc" OPENCV_VERSION ".lib")
#pragma comment(lib, "opencv_highgui" OPENCV_VERSION ".lib")
#pragma comment(lib, "opencv_video" OPENCV_VERSION ".lib")
#endif    // USE_CMAKE_LIBS
#endif    // CV_VERSION_EPOCH


void draw_boxes(cv::Mat mat_img, std::vector<bbox_t> result_vec, std::vector<std::string> obj_names,
    int current_det_fps = -1, int current_cap_fps = -1)
{
    int const colors[6][3] = { { 1,0,1 },{ 0,0,1 },{ 0,1,1 },{ 0,1,0 },{ 1,1,0 },{ 1,0,0 } };

    for (auto &i : result_vec) {
        cv::Scalar color = obj_id_to_color(i.obj_id);
        cv::rectangle(mat_img, cv::Rect(i.x, i.y, i.w, i.h), color, 2);
        if (obj_names.size() > i.obj_id) {
            std::string obj_name = obj_names[i.obj_id];
            if (i.track_id > 0) obj_name += " - " + std::to_string(i.track_id);
            cv::Size const text_size = getTextSize(obj_name, cv::FONT_HERSHEY_COMPLEX_SMALL, 1.2, 2, 0);
            int max_width = (text_size.width > i.w + 2) ? text_size.width : (i.w + 2);
            max_width = std::max(max_width, (int)i.w + 2);
            //max_width = std::max(max_width, 283);
            std::string coords_3d;
            if (!std::isnan(i.z_3d)) {
                std::stringstream ss;
                ss << std::fixed << std::setprecision(2) << "x:" << i.x_3d << "m y:" << i.y_3d << "m z:" << i.z_3d << "m ";
                coords_3d = ss.str();
                cv::Size const text_size_3d = getTextSize(ss.str(), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, 1, 0);
                int const max_width_3d = (text_size_3d.width > i.w + 2) ? text_size_3d.width : (i.w + 2);
                if (max_width_3d > max_width) max_width = max_width_3d;
            }

            cv::rectangle(mat_img, cv::Point2f(std::max((int)i.x - 1, 0), std::max((int)i.y - 35, 0)),
                cv::Point2f(std::min((int)i.x + max_width, mat_img.cols - 1), std::min((int)i.y, mat_img.rows - 1)),
                color, CV_FILLED, 8, 0);
            putText(mat_img, obj_name, cv::Point2f(i.x, i.y - 16), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.2, cv::Scalar(0, 0, 0), 2);
            if(!coords_3d.empty()) putText(mat_img, coords_3d, cv::Point2f(i.x, i.y-1), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(0, 0, 0), 1);
        }
    }
    if (current_det_fps >= 0 && current_cap_fps >= 0) {
        std::string fps_str = "FPS detection: " + std::to_string(current_det_fps) + "   FPS capture: " + std::to_string(current_cap_fps);
        putText(mat_img, fps_str, cv::Point2f(10, 20), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.2, cv::Scalar(50, 255, 0), 2);
    }
}
#endif    // OPENCV

// ROS Message init
jsk_recognition_msgs::BoundingBox bbox;
jsk_recognition_msgs::BoundingBoxArray bbox_array;
void publish_detection_result(std::vector<bbox_t> const result_vec, std::vector<std::string> const obj_names, ros::Publisher &bbox_array_pub) {
    if (result_vec.size() == 0) { return; }
    bbox_array.header.frame_id = "center_cam";
    for (auto &i : result_vec) {
        if (obj_names.size() > i.obj_id) std::cout << obj_names[i.obj_id] << " - ";
        std::cout << "obj_id = " << i.obj_id << ", track_id = " << i.track_id << ", x = " << i.x << ", y = " << i.y
            << ", w = " << i.w << ", h = " << i.h
            << std::setprecision(3) << ", prob = " << i.prob << std::endl;
        bbox.label = i.obj_id;
        bbox.value = i.track_id;
        // bbox.header.stamp = raw_cloud.header.stamp
        bbox.header.frame_id = "center_cam";
        bbox.pose.position.x = i.x;
        bbox.pose.position.y = i.y;
        bbox.dimensions.x = i.w;
        bbox.dimensions.y = i.h;

        bbox_array.boxes.push_back(bbox);
    }
    bbox_array_pub.publish(bbox_array);
    bbox_array.boxes.clear();
}

void show_console_result(std::vector<bbox_t> const result_vec, std::vector<std::string> const obj_names, int frame_id = -1) {
    if (frame_id >= 0) std::cout << " Frame: " << frame_id << std::endl;
    for (auto &i : result_vec) {
        if (obj_names.size() > i.obj_id) std::cout << obj_names[i.obj_id] << " - ";
        std::cout << "obj_id = " << i.obj_id << ", track_id = " << i.track_id << ", x = " << i.x << ", y = " << i.y
            << ", w = " << i.w << ", h = " << i.h
            << std::setprecision(3) << ", prob = " << i.prob << std::endl;
    }
}

std::vector<std::string> objects_names_from_file(std::string const filename) {
    std::ifstream file(filename);
    std::vector<std::string> file_lines;
    if (!file.is_open()) return file_lines;
    for(std::string line; getline(file, line);) file_lines.push_back(line);
    std::cout << "object names loaded \n";
    return file_lines;
}

template<typename T>
class send_one_replaceable_object_t {
    const bool sync;
    std::atomic<T *> a_ptr;
public:

    void send(T const& _obj) {
        T *new_ptr = new T;
        *new_ptr = _obj;
        if (sync) {
            while (a_ptr.load()) std::this_thread::sleep_for(std::chrono::milliseconds(3));
        }
        std::unique_ptr<T> old_ptr(a_ptr.exchange(new_ptr));
    }

    T receive() {
        std::unique_ptr<T> ptr;
        do {
            while(!a_ptr.load()) std::this_thread::sleep_for(std::chrono::milliseconds(3));
            ptr.reset(a_ptr.exchange(NULL));
        } while (!ptr);
        T obj = *ptr;
        return obj;
    }

    bool is_object_present() {
        return (a_ptr.load() != NULL);
    }

    send_one_replaceable_object_t(bool _sync) : sync(_sync), a_ptr(NULL)
    {}
};

// Flag to monitor if the image was received
bool received_img = false;
int counter = 0;

// For Undistortion
// Mat cameraMatrix = (Mat1d(3, 3) << 284.935394287109, 0, 419.903900146484, 0, 286.006896972656, 402.18359375, 0, 0, 1);
// Mat distortionCoefficients = (Mat1d(1, 4) << 0.00197836104780436, 0.0289195198565722, -0.0268402807414532, 0.0029021711088717);
void ImageCallback(const sensor_msgs::ImageConstPtr& image_msg){
/*
---- ROS Image Callback ----
Objective: Converts sensor_msgs::Image to cv::Mat
*/
        std::cout << "Ran " << counter << "times" << std::endl;
        try
        {
            // Undistort image here
            // temp_openCvImage = cv_bridge::toCvShare(image_msg, "bgr8")->image;
            // cv::fisheye::undistortImage(temp_openCvImage,openCvImage,cameraMatrix,distortionCoefficients,cameraMatrix,Size(848,800));
            openCvImage = cv_bridge::toCvShare(image_msg, "bgr8")->image;
            received_img = true;
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", image_msg->encoding.c_str());
        }
}


int gpu_instance_id = 0;


int main(int argc, char *argv[])
{
    auto start_while = std::chrono::steady_clock::now();
    auto end_while = std::chrono::steady_clock::now();
    float spent_while = std::chrono::duration<double>(end_while - start_while).count();
    std::string  names_file = "data/coco.names";
    std::string  cfg_file = "cfg/yolov3.cfg";
    std::string  weights_file = "yolov3.weights";
    std::string filename;

    if (argc > 4) {    //voc.names yolo-voc.cfg yolo-voc.weights test.mp4
        names_file = argv[1];
        cfg_file = argv[2];
        weights_file = argv[3];
        filename = argv[4];
    }
    else if (argc > 1) filename = argv[1];

// Changed by NEEL: change thresh to 0.2 to revert to original (THIS IS THE DETECTION THRESHOLD)
    
    float const thresh = (argc > 5 && !strcmp(argv[5], "--thresh")) ? std::stof(argv[6]) : 0.5;

    gpu_instance_id = (argc > 5 && !strcmp(argv[5], "--i")) ? std::stoi(argv[6]) : 0;

    Detector detector(cfg_file, weights_file);

    auto obj_names = objects_names_from_file(names_file);
    std::string out_videofile = "result.avi";
    bool const save_output_videofile = false;   // true - for history
    bool const send_network = false;        // true - for remote detection
    bool const use_kalman_filter = false;   // true - for stationary camera

    bool detection_sync = true;             // true - for video-file
#ifdef TRACK_OPTFLOW    // for slow GPU
    detection_sync = false;
    Tracker_optflow tracker_flow;
    //detector.wait_stream = true;
#endif  // TRACK_OPTFLOW

// initialize basler camera
#ifdef BASLER_CAMERA

    // Init node and create node handler
    ros::init(argc, argv, "Yolo_v4");
    ros::NodeHandle nh;

    // Check Boost Version for potential image_transport conflicts
    cout << "\n-------------------------------" << endl;
    std::cout << "Using Boost "     
          << BOOST_VERSION / 100000     << "."  // major version
          << BOOST_VERSION / 100 % 1000 << "."  // minor version
          << BOOST_VERSION % 100                // patch level
          << std::endl;

    // Need to use image_transport to create a subscriber to image topic rather than ros::Subscriber
    cout << "ROS Library Link Test:" << endl;
    std::string path = ros::package::getPath("cv_bridge");
    cout << "cv_bridge is at: " << path << endl;


    //string image_topic = "/pylon_camera_node_left/image_rect";
    string image_topic = "/carla/camera/rgb/node1_camera_right/image_color";
    // string image_topic = "/pylon_camera_node_center/image_rect";

    image_transport::ImageTransport it(nh);

#if isStreamCompressed
    image_transport::TransportHints hints("compressed");
    image_transport::Subscriber image_sub = it.subscribe(image_topic, 1, ImageCallback, ros::VoidPtr(), hints);
#else
    image_transport::Subscriber image_sub = it.subscribe(image_topic, 1, ImageCallback);
#endif

    ros::Publisher bbox_array_pub = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("bbox_array", 1);


    // Wait until communication with ROS master is established
    start_while = std::chrono::steady_clock::now();
    while (!received_img){
        ros::spinOnce();
        end_while = std::chrono::steady_clock::now();
        float spent_while = std::chrono::duration_cast<std::chrono::seconds>(end_while - start_while).count();
        if (spent_while > 2){
            cout << "\n" << "ROS topic " << image_topic << " is not yet published \nExiting..." << endl;
            exit(0);
        }
    }
    received_img = false;   // Set received image status to false again

    // DEBUG:
    int count_enter = 0;

#endif

    while (true)
    {
        std::cout << "input image or video filename: ";
        cout << filename << endl;
        if(filename.size() == 0) std::cin >> filename;
        if (filename.size() == 0) break;

        try {
#ifdef OPENCV
            preview_boxes_t large_preview(100, 150, false), small_preview(50, 50, true);
            bool show_small_boxes = false;

            std::string const file_ext = filename.substr(filename.find_last_of(".") + 1);
            std::string const protocol = filename.substr(0, 7);
            if (file_ext == "avi" || file_ext == "mp4" || file_ext == "mjpg" || file_ext == "mov" ||     // video file
                protocol == "rtmp://" || protocol == "rtsp://" || protocol == "http://" || protocol == "https:/" ||    // video network stream
                filename == "zed_camera" || filename == "basler_camera" || file_ext == "svo" || filename == "web_camera")   // ZED stereo camera

            {
                if (protocol == "rtsp://" || protocol == "http://" || protocol == "https:/" || filename == "zed_camera" || filename == "basler_camera" || filename == "web_camera")
                    detection_sync = false; // *** Was false, but it was publishing bounding box at ~ 50 ish Hz as the thread for detect is running at different fps ***

                cv::Mat cur_frame;
                std::atomic<int> fps_cap_counter(0), fps_det_counter(0);
                std::atomic<int> current_fps_cap(0), current_fps_det(0);
                std::atomic<bool> exit_flag(false);
                std::chrono::steady_clock::time_point steady_start, steady_end;
                int video_fps = 25;
                bool use_zed_camera = false;
                bool use_basler_camera = false;

                track_kalman_t track_kalman;

#ifdef ZED_STEREO
                sl::InitParameters init_params;
                init_params.depth_minimum_distance = 0.5;
    #ifdef ZED_STEREO_2_COMPAT_MODE
                init_params.depth_mode = sl::DEPTH_MODE_ULTRA;
                init_params.camera_resolution = sl::RESOLUTION_HD720;// sl::RESOLUTION_HD1080, sl::RESOLUTION_HD720
                init_params.coordinate_units = sl::UNIT_METER;
                init_params.camera_buffer_count_linux = 2;
                if (file_ext == "svo") init_params.svo_input_filename.set(filename.c_str());
    #else
                init_params.depth_mode = sl::DEPTH_MODE::ULTRA;
                init_params.camera_resolution = sl::RESOLUTION::HD720;// sl::RESOLUTION::HD1080, sl::RESOLUTION::HD720
                init_params.coordinate_units = sl::UNIT::METER;
                if (file_ext == "svo") init_params.input.setFromSVOFile(filename.c_str());
    #endif
                //init_params.sdk_cuda_ctx = (CUcontext)detector.get_cuda_context();
                init_params.sdk_gpu_id = detector.cur_gpu_id;

                if (filename == "zed_camera" || file_ext == "svo") {
                    std::cout << "ZED 3D Camera " << zed.open(init_params) << std::endl;
                    if (!zed.isOpened()) {
                        std::cout << " Error: ZED Camera should be connected to USB 3.0. And ZED_SDK should be installed. \n";
                        getchar();
                        return 0;
                    }
                    cur_frame = zed_capture_rgb(zed);
                    use_zed_camera = true;
                }
#endif  // ZED_STEREO

#ifdef BASLER_CAMERA
                // Do no proceed until the image is received
                // ros::spinOnce() calls the callbacks in a seperate thread
                // so if we do not wait for the image to be received in the
                // main thread, the main thread will proceed with empty images
                // at times
                start_while = std::chrono::steady_clock::now();
                while (!received_img){
                    ros::spinOnce();
                    end_while = std::chrono::steady_clock::now();
                    float spent_while = std::chrono::duration_cast<std::chrono::seconds>(end_while - start_while).count();
                    if (spent_while > 0.5){
                        cout << "\n" << "Process Terminated By User" << endl;
                        cv::destroyWindow("window name");
                        exit(0);
                    }
                }
                received_img = false;   // Set received image status to false again

                cur_frame = openCvImage;
                use_basler_camera = true;
#endif

                cv::VideoCapture cap;
                if (filename == "web_camera") {
                    cap.open(0);
                    cap >> cur_frame;
                } else if (!(use_zed_camera || use_basler_camera)) {
                    cap.open(filename);
                    cap >> cur_frame;
                }
#ifdef CV_VERSION_EPOCH // OpenCV 2.x
                video_fps = cap.get(CV_CAP_PROP_FPS);
#else
                video_fps = cap.get(cv::CAP_PROP_FPS);
#endif
                cv::Size const frame_size = cur_frame.size();
                //cv::Size const frame_size(cap.get(CV_CAP_PROP_FRAME_WIDTH), cap.get(CV_CAP_PROP_FRAME_HEIGHT));
                std::cout << "\n Video size: " << frame_size << std::endl;

                cv::VideoWriter output_video;
                if (save_output_videofile)
#ifdef CV_VERSION_EPOCH // OpenCV 2.x
                    output_video.open(out_videofile, CV_FOURCC('D', 'I', 'V', 'X'), std::max(35, video_fps), frame_size, true);
#else
                    output_video.open(out_videofile, cv::VideoWriter::fourcc('D', 'I', 'V', 'X'), std::max(35, video_fps), frame_size, true);
#endif

                struct detection_data_t {
                    cv::Mat cap_frame;
                    std::shared_ptr<image_t> det_image;
                    std::vector<bbox_t> result_vec;
                    cv::Mat draw_frame;
                    bool new_detection;
                    uint64_t frame_id;
                    bool exit_flag;
                    cv::Mat zed_cloud;
                    std::queue<cv::Mat> track_optflow_queue;
                    detection_data_t() : exit_flag(false), new_detection(false) {}
                };

                const bool sync = detection_sync; // sync data exchange
                send_one_replaceable_object_t<detection_data_t> cap2prepare(sync), cap2draw(sync),
                    prepare2detect(sync), detect2draw(sync), draw2show(sync), draw2write(sync), draw2net(sync);

                std::thread t_cap, t_prepare, t_detect, t_post, t_draw, t_write, t_network;

                // capture new video-frame
                if (t_cap.joinable()) t_cap.join();
                t_cap = std::thread([&]()
                {
                    uint64_t frame_id = 0;
                    detection_data_t detection_data;
                    do {
                        detection_data = detection_data_t();
#ifdef BASLER_CAMERA

                start_while = std::chrono::steady_clock::now();
                while (!received_img){
                    ros::spinOnce();
                    end_while = std::chrono::steady_clock::now();
                    float spent_while = std::chrono::duration_cast<std::chrono::seconds>(end_while - start_while).count();
                    if (spent_while > 0.5){
                        cout << "\n" << "Process Terminated By User" << endl;
                        cv::destroyWindow("window name");
                        exit(0);
                    }
                }
                received_img = false;

                detection_data.cap_frame = openCvImage;
#endif

#ifdef ZED_STEREO
                        if (use_zed_camera) {
                            while (zed.grab() !=
        #ifdef ZED_STEREO_2_COMPAT_MODE
                                sl::SUCCESS
        #else
                                sl::ERROR_CODE::SUCCESS
        #endif
                                ) std::this_thread::sleep_for(std::chrono::milliseconds(2));
                            detection_data.cap_frame = zed_capture_rgb(zed);
                            detection_data.zed_cloud = zed_capture_3d(zed);
                        }
                        else
#endif   // ZED_STEREO
                        if (!use_basler_camera){
                            cap >> detection_data.cap_frame;
                        }
                        fps_cap_counter++;
                        detection_data.frame_id = frame_id++;
                        if (detection_data.cap_frame.empty() || exit_flag) {
                            std::cout << " exit_flag: detection_data.cap_frame.size = " << detection_data.cap_frame.size() << std::endl;
                            detection_data.exit_flag = true;
                            detection_data.cap_frame = cv::Mat(frame_size, CV_8UC3);
                        }

                        if (!detection_sync) {
                            cap2draw.send(detection_data);       // skip detection
                        }
                        cap2prepare.send(detection_data);
                    } while (!detection_data.exit_flag);
                    std::cout << " t_cap exit \n";
                });


                // pre-processing video frame (resize, convertion)
                t_prepare = std::thread([&]()
                {
                    std::shared_ptr<image_t> det_image;
                    detection_data_t detection_data;
                    do {
                        detection_data = cap2prepare.receive();

                        det_image = detector.mat_to_image_resize(detection_data.cap_frame);
                        detection_data.det_image = det_image;
                        prepare2detect.send(detection_data);    // detection

                    } while (!detection_data.exit_flag);
                    std::cout << " t_prepare exit \n";
                });


                // detection by Yolo
                if (t_detect.joinable()) t_detect.join();
                t_detect = std::thread([&]()
                {
                    std::shared_ptr<image_t> det_image;
                    detection_data_t detection_data;
                    do {
                        detection_data = prepare2detect.receive();
                        det_image = detection_data.det_image;
                        std::vector<bbox_t> result_vec;

                        if(det_image)
                            result_vec = detector.detect_resized(*det_image, frame_size.width, frame_size.height, thresh, true);  // true
                        fps_det_counter++;
                        //std::this_thread::sleep_for(std::chrono::milliseconds(150));

                        detection_data.new_detection = true;
                        detection_data.result_vec = result_vec;
                        detect2draw.send(detection_data);
                    } while (!detection_data.exit_flag);
                    std::cout << " t_detect exit \n";
                });

                // draw rectangles (and track objects)
                t_draw = std::thread([&]()
                {
                    std::queue<cv::Mat> track_optflow_queue;
                    detection_data_t detection_data;
                    do {

                        // for Video-file
                        if (detection_sync) {
                            detection_data = detect2draw.receive();
                        }
                        // for Video-camera
                        else
                        {
                            // get new Detection result if present
                            if (detect2draw.is_object_present()) {
                                cv::Mat old_cap_frame = detection_data.cap_frame;   // use old captured frame
                                detection_data = detect2draw.receive();
                                if (!old_cap_frame.empty()) detection_data.cap_frame = old_cap_frame;
                            }
                            // get new Captured frame
                            else {
                                std::vector<bbox_t> old_result_vec = detection_data.result_vec; // use old detections
                                detection_data = cap2draw.receive();
                                detection_data.result_vec = old_result_vec;
                            }
                        }

                        cv::Mat cap_frame = detection_data.cap_frame;
                        cv::Mat draw_frame = detection_data.cap_frame.clone();
                        std::vector<bbox_t> result_vec = detection_data.result_vec;

#ifdef TRACK_OPTFLOW
                        if (detection_data.new_detection) {
                            tracker_flow.update_tracking_flow(detection_data.cap_frame, detection_data.result_vec);
                            while (track_optflow_queue.size() > 0) {
                                draw_frame = track_optflow_queue.back();
                                result_vec = tracker_flow.tracking_flow(track_optflow_queue.front(), false);
                                track_optflow_queue.pop();
                            }
                        }
                        else {
                            track_optflow_queue.push(cap_frame);
                            result_vec = tracker_flow.tracking_flow(cap_frame, false);
                        }
                        detection_data.new_detection = true;    // to correct kalman filter
#endif //TRACK_OPTFLOW

                        // track ID by using kalman filter
                        if (use_kalman_filter) {
                            if (detection_data.new_detection) {
                                result_vec = track_kalman.correct(result_vec);
                            }
                            else {
                                result_vec = track_kalman.predict();
                            }
                        }
                        // track ID by using custom function
                        else {
                            int frame_story = std::max(5, current_fps_cap.load());
                            result_vec = detector.tracking_id(result_vec, true, frame_story, 40);
                        }

                        if (use_zed_camera && !detection_data.zed_cloud.empty()) {
                            result_vec = get_3d_coordinates(result_vec, detection_data.zed_cloud);
                        }

                        //small_preview.set(draw_frame, result_vec);
                        //large_preview.set(draw_frame, result_vec);
                        draw_boxes(draw_frame, result_vec, obj_names, current_fps_det, current_fps_cap);
                        // publish_detection_result(result_vec, obj_names, bbox_array_pub);
                        // show_console_result(result_vec, obj_names, detection_data.frame_id);
                        //large_preview.draw(draw_frame);
                        //small_preview.draw(draw_frame, true);

                        detection_data.result_vec = result_vec;
                        detection_data.draw_frame = draw_frame;
                        draw2show.send(detection_data);
                        if (send_network) draw2net.send(detection_data);
                        if (output_video.isOpened()) draw2write.send(detection_data);
                    } while (!detection_data.exit_flag);
                    std::cout << " t_draw exit \n";
                });


                // write frame to videofile
                t_write = std::thread([&]()
                {
                    if (output_video.isOpened()) {
                        detection_data_t detection_data;
                        cv::Mat output_frame;
                        do {
                            detection_data = draw2write.receive();
                            if(detection_data.draw_frame.channels() == 4) cv::cvtColor(detection_data.draw_frame, output_frame, CV_RGBA2RGB);
                            else output_frame = detection_data.draw_frame;
                            output_video << output_frame;
                        } while (!detection_data.exit_flag);
                        output_video.release();
                    }
                    std::cout << " t_write exit \n";
                });

                // send detection to the network
                t_network = std::thread([&]()
                {
                    if (send_network) {
                        detection_data_t detection_data;
                        do {
                            detection_data = draw2net.receive();

                            detector.send_json_http(detection_data.result_vec, obj_names, detection_data.frame_id, filename);

                        } while (!detection_data.exit_flag);
                    }
                    std::cout << " t_network exit \n";
                });
                namedWindow("window name", CV_WINDOW_NORMAL);
                resizeWindow("window name", 848, 800);
                // show detection
                detection_data_t detection_data;
                do {

                    steady_end = std::chrono::steady_clock::now();
                    float time_sec = std::chrono::duration<double>(steady_end - steady_start).count();
                    if (time_sec >= 1) {
                        current_fps_det = fps_det_counter.load() / time_sec;
                        current_fps_cap = fps_cap_counter.load() / time_sec;
                        steady_start = steady_end;
                        fps_det_counter = 0;
                        fps_cap_counter = 0;
                    }

                    detection_data = draw2show.receive();
                    cv::Mat draw_frame = detection_data.draw_frame;

                    //if (extrapolate_flag) {
                    //    cv::putText(draw_frame, "extrapolate", cv::Point2f(10, 40), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, cv::Scalar(50, 50, 0), 2);
                    //}
                    publish_detection_result(detection_data.result_vec, obj_names, bbox_array_pub);
                    cv::imshow("window name", draw_frame);
                    int key = cv::waitKey(1);    // 3 or 16ms
                    if (key == 'f') show_small_boxes = !show_small_boxes;
                    if (key == 'p') while (true) if (cv::waitKey(100) == 'p') break;
                    // if (key == 'e') extrapolate_flag = !extrapolate_flag;
                    if (key == 27) { exit_flag = true;}

                    std::cout << " current_fps_det = " << current_fps_det << ", current_fps_cap = " << current_fps_cap << std::endl;
                } while (!detection_data.exit_flag);
                std::cout << " show detection exit \n";

                cv::destroyWindow("window name");
                // wait for all threads
                if (t_cap.joinable()) t_cap.join();
                if (t_prepare.joinable()) t_prepare.join();
                if (t_detect.joinable()) t_detect.join();
                if (t_post.joinable()) t_post.join();
                if (t_draw.joinable()) t_draw.join();
                if (t_write.joinable()) t_write.join();
                if (t_network.joinable()) t_network.join();

                break;

            }
            else if (file_ext == "txt") {    // list of image files
                std::ifstream file(filename);
                if (!file.is_open()) std::cout << "File not found! \n";
                else
                    for (std::string line; file >> line;) {
                        std::cout << line << std::endl;
                        cv::Mat mat_img = cv::imread(line);
                        std::vector<bbox_t> result_vec = detector.detect(mat_img);
                        show_console_result(result_vec, obj_names);
                        //draw_boxes(mat_img, result_vec, obj_names);
                        //cv::imwrite("res_" + line, mat_img);
                    }

            }
            else {    // image file
                // to achive high performance for multiple images do these 2 lines in another thread
                cv::Mat mat_img = cv::imread(filename);
                auto det_image = detector.mat_to_image_resize(mat_img);

                auto start = std::chrono::steady_clock::now();
                std::vector<bbox_t> result_vec = detector.detect_resized(*det_image, mat_img.size().width, mat_img.size().height);
                auto end = std::chrono::steady_clock::now();
                std::chrono::duration<double> spent = end - start;
                std::cout << " Time: " << spent.count() << " sec \n";

                //result_vec = detector.tracking_id(result_vec);    // comment it - if track_id is not required
                draw_boxes(mat_img, result_vec, obj_names);
                cv::imshow("window name", mat_img);
                show_console_result(result_vec, obj_names);
                cv::waitKey(0);
            }
#else   // OPENCV
            //std::vector<bbox_t> result_vec = detector.detect(filename);

            auto img = detector.load_image(filename);
            std::vector<bbox_t> result_vec = detector.detect(img);
            detector.free_image(img);
            show_console_result(result_vec, obj_names);
#endif  // OPENCV
        }
        catch (std::exception &e) { std::cerr << "exception: " << e.what() << "\n"; getchar(); }
        catch (...) { std::cerr << "unknown exception \n"; getchar(); }
        filename.clear();
    }

    return 0;
}
