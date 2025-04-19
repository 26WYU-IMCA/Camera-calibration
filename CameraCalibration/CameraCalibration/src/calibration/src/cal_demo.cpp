#include "/home/zz/CameraCalibration/src/calibration/include/cal.hpp"
Mat Calibrator::cvtColors(Mat img){
    if(img.channels() == 1){
        return img;
    }
    else{
    Mat gray_img;
    cvtColor(img, gray_img, COLOR_BGR2GRAY);
    return gray_img;
    }
}
bool Calibrator::findChessboardCorner(Mat gray_img, Size board_size, vector<Point2f>& corners){
    bool found = findChessboardCorners(gray_img, board_size, corners);
    return found;
}
void Calibrator::calibrateCameration(Mat gray_img, vector<Point2f>&corners, vector<vector<Point3f>>& object_points, vector<vector<Point2f>>& image_points, int board_width, int board_height, double square_size){
    cv::cornerSubPix(gray_img, corners, Size(11, 11), Size(-1, -1), TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
    image_points.push_back(corners);
    vector<Point3f> obj;
    Point3fs(object_points, obj, board_width, board_height, square_size);
}
void Calibrator::Point3fs(vector<vector<Point3f>>& object_points, vector<Point3f>& obj, int board_width, int board_height, double square_size){
  for (int i = 0; i < board_height; i++){
        for (int j = 0; j < board_width; j++){
        obj.push_back(Point3f(j * square_size, i * square_size, 0));
        }
    }
    object_points.push_back(obj);
}
Mat Calibrator::draw(Mat img, Size board_size, vector<Point2f>& corners,bool found){
    drawChessboardCorners(img, board_size, corners, found);
    return img;
}
