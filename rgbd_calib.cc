#include <calib.h>
#include <iostream>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

int main(int argc, char *argv[])
{
    std::string rgddir, depthdir;
    std::string root_dir, param_dir, param_path;
    std::string sensor_type, chessboard_type;
    cv::Size board_size(0, 0);
    cv::Size radius_size(5, 5);
    float square_size;
    bool verbose, extrincs_save;

    boost::program_options::options_description desc("monocular calibration");
    desc.add_options()
    ("help", "produce help message")
    ("rgb-dir", boost::program_options::value<std::string>(&rgddir), "the dir of rgb images(required)")
    ("depth-dir", boost::program_options::value<std::string>(&depthdir), "the dir of depth (required)")
    ("board-height,h", boost::program_options::value<int>(&board_size.height), "棋盘格规格的高(required)")
    ("board-width,w", boost::program_options::value<int>(&board_size.width), "棋盘格规格的宽(required)")
    ("sensor-type", boost::program_options::value<std::string>(&sensor_type)->default_value("Pinhole"), "传感器类型(Pinhole或者Fisheye)")
    ("chessboard-type", boost::program_options::value<std::string>(&chessboard_type)->default_value("Corner"), "棋盘格类型(Corner或者Circle)")
    ("square-size,s", boost::program_options::value<float>(&square_size), "棋盘格小格子尺寸(required)")
    ("radius-height", boost::program_options::value<int>(&radius_size.height)->default_value(5), "亚像素角点查找的高度半径")
    ("radius-width", boost::program_options::value<int>(&radius_size.width)->default_value(5), "亚像素角点查找的宽度半径")
    ("output-dir,o", boost::program_options::value<std::string>(&param_dir)->default_value("."),"保存目录路径")
    ("verbose,v", boost::program_options::bool_switch(&verbose)->default_value(false), "是否查看具体的内容")
    ;

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
    std::cout << "图像根目录 : " << root_dir << std::endl
              << "相机传感器类别 : " << sensor_type << std::endl
              << "标定板类别 : " << chessboard_type << std::endl;
    calibration::Calib::CameraSensorType SensorType = sensor_type == "Pinhole" ? calibration::Calib::CameraSensorType::Pinhole : calibration::Calib::CameraSensorType::Fisheye;
    calibration::Calib::ChessboardType ChessboardType = chessboard_type == "Corner" ? calibration::Calib::ChessboardType::CornerGrid : calibration::Calib::ChessboardType::CircleGrid;
    // for rgb
    std::shared_ptr<calibration::Calib> rgb_calib = calibration::Calib::create(
        SensorType,
        ChessboardType,
        rgddir,
        board_size,
        square_size,
        radius_size);
    rgb_calib->setflagshow(verbose);
    rgb_calib->setflagextrincs(true);
    rgb_calib->run();

    // for depth
    std::shared_ptr<calibration::Calib> depth_calib = calibration::Calib::create(
        SensorType,
        ChessboardType,
        depthdir,
        board_size,
        square_size,
        radius_size);
    depth_calib->setflagshow(verbose);
    depth_calib->setflagextrincs(true);
    depth_calib->run();
    // get intrincs
    cv::Mat rgbK = rgb_calib->getleftK();
    cv::Mat depthK = depth_calib->getleftK();
    // get extrincs
    std::vector<cv::Mat> rgb_rvecs = rgb_calib->getleftrvecs();
    std::vector<cv::Mat> depth_rvecs = depth_calib->getleftrvecs();
    std::vector<cv::Mat> rgb_tvecs = rgb_calib->getlefttvecs();
    std::vector<cv::Mat> depth_tvecs = depth_calib->getlefttvecs();
    cv::Mat rgbR, depthR;
    cv::Rodrigues(rgb_rvecs[0], rgbR);
    cv::Rodrigues(depth_rvecs[0], depthR);
    std::cout << "the rotation matrix of rgb camera : \n"
              << rgbR
              << "\n the rotation matrix of depth camera : \n"
              << depthR
              << std::endl;
    cv::Mat R  =  rgbR * depthR.t();
    cv::Mat t = rgb_tvecs[0] - R * depth_tvecs[0];
    std::cout << "the rotation matrix from the depth to rgb : \n"
              << R
              << "\n the translation matrix from the depth to rgb : \n"
              << t
              << std::endl;

    if (!param_dir.empty())
    {
        std::cout << "保存参数," << "保存路径为 : " << param_dir << std::endl;
        if (!boost::filesystem::exists(param_dir))
        {
            boost::filesystem::create_directories(param_dir);
        }
        boost::filesystem::path param_path(param_dir);
        cv::FileStorage file((param_path / "Register.yaml").c_str(), cv::FileStorage::WRITE);
        file << "rgbK" << rgbK
             << "depthK" << depthK
             << "R" <<R
             << "t" << t;


        file.release();
        return 1;
    }
    std::cout << "此次不保存参数！" << std::endl;
    return 1;
}