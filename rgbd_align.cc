#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
// 相机内参

void filterblack(const cv::Mat src, cv::Mat &dst)
{
    cv::Mat src_ = src.clone();
    if (src_.channels() == 3)
        cv::cvtColor(src_, src_, cv::COLOR_BGR2GRAY);
    else if (src_.channels() == 4)
        cv::cvtColor(src_, src_, cv::COLOR_BGRA2GRAY);
    cv::Mat mask;
    cv::threshold(src_, mask, 10, 255, cv::THRESH_BINARY_INV);
    cv::Mat kernel = cv::getStructuringElement(0, cv::Size(1, 3));
    cv::Mat di_mask;
    cv::dilate(mask, di_mask, kernel);
    cv::inpaint(src, di_mask, dst, 3, cv::INPAINT_TELEA);
}

int main(int argc, char *argv[])
{
    std::string rgb_image, depth_image;
    std::string param_path;

    boost::program_options::options_description desc("monocular calibration");
    desc.add_options()
    ("help,h", "produce help message")
    ("rgb-img,r", boost::program_options::value<std::string>(&rgb_image), "the path of rgb img(required)")
    ("depth-img,d", boost::program_options::value<std::string>(&depth_image), "the path of depth img(required)")
    ("param,p", boost::program_options::value<std::string>(&param_path), "the path of parameters(required)");

    // 解析命令行参数
    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
    boost::program_options::notify(vm);

    // 检查是否请求帮助
    if (vm.count("help"))
    {
        std::cout << desc << "\n";
        return 1;
    }

    // read yaml
    cv::FileStorage paramfile(param_path, cv::FileStorage::READ);
    // parameters
    double color_cx, color_cy, color_fx, color_fy;
    double depth_cx, depth_cy, depth_fx, depth_fy;
    // read
    cv::Mat rgbK, depthK;
    paramfile["rgbK"] >> rgbK;
    paramfile["depthK"] >> depthK;
    color_fx = rgbK.at<double>(0, 0);
    color_fy = rgbK.at<double>(1, 1);
    color_cx = rgbK.at<double>(0, 2);
    color_cy = rgbK.at<double>(1, 2);

    depth_fx = depthK.at<double>(0, 0);
    depth_fy = depthK.at<double>(1, 1);
    depth_cx = depthK.at<double>(0, 2);
    depth_cy = depthK.at<double>(1, 2);
    // set data
    cv::Mat color = cv::imread(rgb_image, 1);
    cv::Mat depth = cv::imread(depth_image, -1);
    // show
    cv::imshow("rgb img",color);
    cv::imshow("depth img",depth * 8);
    cv::waitKey();
    cv::destroyAllWindows();
    
    // for aligned data
    cv::Mat aligned_depth(depth.size(), CV_32FC1, cv::Scalar(0));
    cv::Mat aligned_color = color.clone();
    cv::resize(aligned_color, aligned_color, depth.size());
    cv::Mat color_vis(depth.size(), CV_8UC3, cv::Scalar(0));
    // scale factor
    double scale_x, scale_y;
    scale_x = (double)depth.rows / color.rows;
    scale_y = (double)depth.cols / color.cols;
    // point cloud
    std::ofstream outfile("cloud.txt");

    // convert
    cv::Mat cvR,cvt;
    paramfile["R"] >> cvR;
    paramfile["t"] >> cvt;
    Eigen::Matrix3f R = Eigen::Matrix<double,3,3,Eigen::RowMajor>((double*)cvR.data).cast<float>();
    Eigen::Vector3f T = Eigen::Vector3d((double*) cvt.data).cast<float>(); // d2c平移向量
    std::cout<< "R : \n"<<cvR<<std::endl;
    std::cout<< "t : \n"<<cvt<<std::endl;
    int point_nums = 0;
    for (int i = 0; i < depth.rows; ++i)
    {
        for (int j = 0; j < depth.cols; ++j)
        {
            ushort d = depth.ptr<ushort>(i)[j];

            if (d == 0)
                continue;

            point_nums++;
            Eigen::Vector3f pRGB,pPoint;
            pPoint.z() = double(d);
            pPoint.x() = (j - depth_cx) * pPoint.z() / depth_fx;
            pPoint.y() = (i - depth_cy) * pPoint.z() / depth_fy;
            // transform
            Eigen::Vector3f Pc = R * pPoint + T;
            int n = Pc.x() * color_fx / Pc.z() + color_cx;
            if (n < 0)
                n = 0;
            if (n >= color.cols)
                n = color.cols - 1;

            int m = Pc.y() * color_fy / Pc.z() + color_cy;
            if (m < 0)
                m = 0;
            if (m >= color.rows)
                m = color.rows - 1;

            pRGB.z() = color.at<cv::Vec3b>(m, n)[0];
            pRGB.y() = color.at<cv::Vec3b>(m, n)[1];
            pRGB.x() = color.at<cv::Vec3b>(m, n)[2];
            // save data
            outfile << pPoint.x() << " " << pPoint.y() << " " << pPoint.z() << " " << (int)pRGB.x() << " " << (int)pRGB.y() << " " << (int)pRGB.z() << std::endl;
            // for aligned depth image
            aligned_depth.at<float>(int(m * scale_x), int(n * scale_y)) = (float)pPoint.z();
            // for color vis
            color_vis.at<cv::Vec3b>(int(m * scale_x), int(n * scale_y)) = color.at<cv::Vec3b>(m, n);
        }
    }

    double min, max;
    cv::minMaxLoc(aligned_depth, &min, &max);
    cv::Mat depth_vis;

    // normlize
    aligned_depth = (aligned_depth - min) / (max - min) * 255;
    aligned_depth.convertTo(depth_vis, CV_8U);
    cv::cvtColor(depth_vis, depth_vis, cv::COLOR_GRAY2BGR);
    
    // to clear the line
    cv::medianBlur(depth_vis, depth_vis, 5);
    cv::medianBlur(color_vis, color_vis, 5);

    // mix the color image and depth visualization
    cv::Mat mixture;
    cv::addWeighted(aligned_color, 0.3, depth_vis, 0.7, 0, mixture);
    
    // show time
    cv::imshow("aligned rgb", aligned_color);
    cv::imshow("aligned depth", depth_vis);
    cv::waitKey();
    cv::destroyAllWindows();
    cv::imshow("mixture", mixture);
    cv::imshow("color vis", color_vis);
    cv::waitKey();
    cv::destroyAllWindows();
    cv::imwrite("vis.png", color_vis);
    outfile.close();

    return 0;
}