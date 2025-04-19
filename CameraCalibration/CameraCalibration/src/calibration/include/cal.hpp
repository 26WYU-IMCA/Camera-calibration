//std
#include <opencv2/opencv.hpp>
//opencv
#include <iostream>
//ros2
#include <rclcpp/rclcpp.hpp>
using namespace cv;
using namespace std;
class Calibrator{
private:
    void Point3fs(vector<vector<Point3f>>& object_points, vector<Point3f>& obj,int board_width, int board_height, double square_size);
public :
    vector<vector<Point3f>> object_points;
    vector<vector<Point2f>> image_points;
    int board_width, board_height;//棋盘格宽高
    double square_size;//棋盘格尺寸
    size_t size__;//标定图片数量
    double times = 0;//间隔时间，默认0
    Size board_size;
    Mat cvtColors(Mat img);//转换为灰度图
    bool findChessboardCorner(Mat gray_img, Size pattern_size, vector<Point2f>& corners);//寻找棋盘格角点
    void calibrateCameration(Mat gray_img, vector<Point2f>&corners, vector<vector<Point3f>>& object_points, vector<vector<Point2f>>& image_points, int board_width, int board_height, double square_size);//标定相机
    Mat draw(Mat img, Size board_size, vector<Point2f>& corners,bool found);//绘制棋盘格
};