#include <iostream>
#include <cmath>

#include "cuda_utils.h"
#include "utils.h"
#include "o3dutils.h"

#include "GLViewer.hpp"

int main(int argc, char **argv)
{

    // Create a ZED camera object
    std::unique_ptr<sl::Camera> ptr_zed = std::make_unique<sl::Camera>();

    // Defines the cam parameters
    sl::InitParameters cam_params;
    cam_params.depth_mode = sl::DEPTH_MODE::ULTRA;
    cam_params.depth_maximum_distance = 5.0f;
    cam_params.coordinate_units = sl::UNIT::METER;
    cam_params.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP;
    cam_params.camera_resolution = sl::RESOLUTION::HD720;
    cam_params.camera_fps = 60.0f;

    // Open the camera
    sl::ERROR_CODE err = ptr_zed->open(cam_params);
    if (err != sl::ERROR_CODE::SUCCESS)
    {
        std::cout << "Error, unable to open ZED camera: " << err << "\n";
        ptr_zed->close();
        return 1; // Quit if an error occurred
    }

    // static camera
    sl::PositionalTrackingParameters positional_tracking_parameters;
    positional_tracking_parameters.set_as_static = true;
    sl::ERROR_CODE state = ptr_zed->enablePositionalTracking(positional_tracking_parameters);
    if (state != sl::ERROR_CODE::SUCCESS)
    {
        std::cout << "Error: " << state << std::endl;
        return false;
    }

    // Start camera threads after creating a shared_ptr to the GLViewer
    std::unique_ptr<GLViewer> ptr_viewer = std::make_unique<GLViewer>();
    ptr_viewer->init(argc, argv);
    std::cout << "Initialized OpenGL Viewer!";

    // // showing cmds
    std::cout << "Viewer Shortcuts\n"
              << "\t- 'r': swicth on/off for raw skeleton display\n"
              << "\t- 'p': swicth on/off for live point cloud display\n"
              << "\t- 'c': swicth on/off point cloud display with flat color\n"
              << std::endl;

    const unsigned int id = ptr_zed->getCameraInformation().serial_number;

    sl::Resolution low_res(512, 360);

    while (ptr_viewer->isAvailable())
    {
        if (ptr_zed->grab() == sl::ERROR_CODE::SUCCESS)
        {
            sl::Pose sl_pose;
            auto state_pose = ptr_zed->getPosition(sl_pose, sl::REFERENCE_FRAME::WORLD);
            if (state_pose == sl::POSITIONAL_TRACKING_STATE::OK)
            {
                ptr_viewer->setCameraPose(id, sl_pose.pose_data);
                // std::cout << "updating Pose" << std::endl;
            }

            sl::Mat sl_pc;

            // cpu
            // auto state_pc = ptr_zed->retrieveMeasure(sl_pc, sl::MEASURE::XYZBGRA, sl::MEM::CPU, low_res);
            // auto o3d_pc = slMat_to_Open3D(sl_pc);
            // auto updated = Open3D_to_slMat(o3d_pc);

            // gpu
            auto state_pc = ptr_zed->retrieveMeasure(sl_pc, sl::MEASURE::XYZBGRA, sl::MEM::CPU, low_res);
            auto o3d_pc = slMat_to_Open3D_gpu(sl_pc);
            auto updated = Open3D_to_slMat_gpu(o3d_pc);
            // auto v_downsampled_pcd = o3d_pc.VoxelDownSample(0.1);
            // auto updated = Open3D_to_slMat_gpu(v_downsampled_pcd);

            ptr_viewer->updatePC(id, updated);
        }
        // rendering
        std::this_thread::sleep_for(std::chrono::microseconds(16));
    }

    ptr_viewer->exit();
    ptr_zed->close();

    return EXIT_SUCCESS;
}