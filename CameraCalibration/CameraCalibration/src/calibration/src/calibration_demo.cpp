//std
#include <iostream>
#include <fstream>
#include <filesystem>
#include <chrono>

//opencv
#include <opencv4/opencv2/opencv.hpp>

//ros2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>

#include "/home/zz/CameraCalibration/src/calibration/include/cal.hpp"
#include <yaml-cpp/yaml.h>

// std
using namespace std;
namespace fs = std::filesystem;
// opencv
using namespace cv;
class CalibrationDemo : public rclcpp::Node{
private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_ ;
    rclcpp::TimerBase::SharedPtr timer__;
    std::shared_ptr<rclcpp::Node> node_;
    //函数
    bool node_detection();
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
    void times_callback();//定时器回调
    bool parameter(int argc, char **argv);//参数处理
    void create_folder();//创建文件夹
    void write_mat(const fs::path &folderpath, vector<Mat> &mats, int mat_i);//写入mat文件
    void write(const fs::path &folderpath,vector<vector<Point2f>>& image_points, vector<vector<Point3f>>& object_points, size_t size__, Mat img);//写入yaml文件
    //变量
    string name_topic;
    Calibrator calibra;
    Mat img;
    vector<Mat> cameras;//相机图像
    string folderName = "camera_info";
    fs::path folderpath;
    chrono::time_point<chrono::system_clock> last_time;
    char a;
public:    
    CalibrationDemo(int argc, char **argv) : Node("calibration_demo"){
        if(!parameter(argc, argv)){
        return;
        }
        create_folder();
        if(!node_detection()){
            return;
        }
        sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(name_topic, 1, std::bind(&CalibrationDemo::image_callback, this, std::placeholders::_1));
        timer__ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&CalibrationDemo::times_callback, this));
        last_time = chrono::system_clock::now();
    }
};
void CalibrationDemo::create_folder(){
    folderpath = fs::path(folderName);
    if (fs::create_directory(folderpath)){
        RCLCPP_INFO(this->get_logger(), "创建文件夹成功");
    }
    else{
      RCLCPP_INFO(this->get_logger(), "文件夹已存在");
    }   
}
bool CalibrationDemo::parameter(int argc, char **argv){
    if (argc != 4 && argc != 5 ){
    RCLCPP_ERROR(this->get_logger(), "参数错误1");
    return false;
    }
    std::string str = argv[1];
    std::istringstream iss(str);
    string token;
    vector<int> prats;
    while (getline(iss, token, 'X')){
    prats.push_back(stoi(token));
    }
    if (prats.size() != 2){
    RCLCPP_ERROR(this->get_logger(), "参数错误2");
    return false;
    }
    calibra.board_width = prats[0];
    calibra.board_height = prats[1];
    calibra.square_size = stod(argv[2]);
    calibra.size__ = stoi(argv[3]);
    if(argc == 5){
      if(stoi(argv[4]) < 0){
        RCLCPP_INFO(this->get_logger(), "参数错误4");
        return false;
      }
      else{
        calibra.times = stoi(argv[4]);
      }
    }
    calibra.board_size = Size(calibra.board_width, calibra.board_height);
    return true;
}       
void CalibrationDemo::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg){

    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    img = cv_ptr->image;
    if(img.empty()){
        return;
    }
}  
void CalibrationDemo::times_callback(){
    Mat gray_img;
    gray_img = calibra.cvtColors(img);
    auto current_time = chrono::system_clock::now();
    auto elapsed_time = chrono::duration_cast<chrono::seconds>(current_time - last_time).count();
    vector<Point2f> corners;
    bool found = calibra.findChessboardCorner(gray_img, calibra.board_size, corners) && elapsed_time >= calibra.times;
    RCLCPP_INFO(this->get_logger(), "找到棋盘格或者时间到了%d", found);
    if(found){
        calibra.calibrateCameration(gray_img, corners, calibra.object_points, calibra.image_points, calibra.board_width, calibra.board_height, calibra.square_size);
        last_time = current_time;
    }
    Mat color_img = calibra.draw(img, calibra.board_size, corners, found);
    if(found){
        cameras.push_back(color_img);
        int mat_i = cameras.size() - 1;
        write_mat(folderpath, cameras, mat_i);
    }
    imshow("camera", color_img);
    char c = waitKey(1);
    if(c == 27){
        write(folderpath, calibra.image_points, calibra.object_points, calibra.size__, img);
        RCLCPP_INFO(this->get_logger(), "esc键退出");
        timer__->cancel();
        cv::destroyAllWindows();
    }
}  
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CalibrationDemo>(argc, argv);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}           
void CalibrationDemo::write_mat(const fs::path &folderpath, vector<Mat> &mats, int mat_i){
    imwrite(folderpath / ("image_"+std::to_string(mat_i) + ".jpg"), mats[mat_i]);
}
void CalibrationDemo::write(const fs::path &folderpath,vector<vector<Point2f>>& image_points, vector<vector<Point3f>>& object_points, size_t size__, Mat img){
   if (image_points.size() > size__){
        cv::Mat camera_matrix, dist_coeffs;
        vector<Mat> rvec, tvec;
        double rep = calibrateCamera(object_points, image_points, img.size(), camera_matrix, dist_coeffs, rvec, tvec);
        RCLCPP_INFO(this->get_logger(), "%f", rep);
        fs::path filepath = folderpath / "data.yaml";
        std::ofstream outFile(filepath.string());
        if (outFile.is_open()){
        outFile << "相机内参矩阵：" << "\n"
              << camera_matrix << endl;
        outFile << "畸变系数：" << "\n"
              << dist_coeffs << endl;
        outFile << "rep:" << "\n"
              << rep << endl;
        outFile.close();
        }
    } 
}
bool CalibrationDemo::node_detection(){
    YAML::Node config = YAML::LoadFile("/home/zz/CameraCalibration/sample/sample.yaml");
    vector<string> topic;
    for(size_t i = 0; i < config["camera_nodes"]["ros__parameters"]["node_names"].size(); i++){
        topic.push_back(config["camera_nodes"]["ros__parameters"]["node_names"][i].as<std::string>());
    }
    auto topic_names = this->get_topic_names_and_types();
    for(auto& n : topic_names){
        RCLCPP_INFO(this->get_logger(), "topic_name:%s", n.first.c_str());
        for(size_t i = 0; i < topic.size(); i++){
            if(n.first == ('/'+topic[i])){
            name_topic = topic[i]; 
            a='a';
            break;
            }
        }
        if(a == 'a'){
            break;
        }
    }
    if(name_topic.empty()){
        RCLCPP_ERROR(this->get_logger(), "未找到相机节点");
        return false;
    }
    return true;
}
