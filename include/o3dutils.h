#ifndef O3D_UTILS_H_
#define O3D_UTILS_H_

#include <open3d/Open3D.h>
#include <sl/Camera.hpp>
#include <Eigen/Dense>
#include <algorithm>

constexpr float RGB_SCALE_FACTOR = 1.0f / 255.0f;

/**
 * https://www.stereolabs.com/docs/depth-sensing/using-depth
 * The point cloud stores its data on 4 channels using a 32-bit float for each channel.
 * The last float is used to store color information, where R, G, B, and alpha channels (4 x 8-bit) are concatenated into a single 32-bit float.
 * */
inline open3d::geometry::PointCloud slMat_to_Open3D(const sl::Mat &input)
{
    // check type? sl::MAT_TYPE::F32_C4
    open3d::geometry::PointCloud pcd;
    int ptsCount = input.getHeight() * input.getWidth();
    auto cloudPtr = input.getPtr<sl::float4>();

    pcd.points_.reserve(ptsCount);
    pcd.colors_.reserve(ptsCount);

    for (int i = 0; i < ptsCount; ++i)
    {
        sl::Vector4<float> *subPtr = &cloudPtr[i];
        if (subPtr->x == subPtr->x and !isinf(subPtr->x))
        {
            pcd.points_.emplace_back(Eigen::Vector3d(subPtr->x, subPtr->y, subPtr->z));

            // Using std::copy for copying color data
            auto colorPtr = reinterpret_cast<uchar *>(&subPtr->w);
            float rgb[3];
            std::copy(colorPtr, colorPtr + 3, rgb);

            // Normalize the RGB values
            std::transform(std::begin(rgb), std::end(rgb), std::begin(rgb), [](float c)
                           { return c * RGB_SCALE_FACTOR; });
            pcd.colors_.emplace_back(Eigen::Vector3d(rgb[0], rgb[1], rgb[2]));
        }
    }
    return pcd;
}

inline open3d::t::geometry::PointCloud slMat_to_Open3D_gpu(const sl::Mat &input)
{
    int inputSize = input.getHeight() * input.getWidth();
    auto cloudPtr = input.getPtr<sl::float4>();
    std::vector<float> pts;
    std::vector<float> clr;
    pts.reserve(inputSize * 3);
    clr.reserve(inputSize * 3);

    for (int i = 0; i < inputSize; ++i)
    {
        sl::Vector4<float> *subPtr = &cloudPtr[i];
        if (subPtr->x == subPtr->x and !isinf(subPtr->x))
        {
            pts.emplace_back(subPtr->x);
            pts.emplace_back(subPtr->y);
            pts.emplace_back(subPtr->z);
            // Using std::copy for copying color data
            auto colorPtr = reinterpret_cast<uchar *>(&subPtr->w);
            float rgb[3];
            std::copy(colorPtr, colorPtr + 3, rgb);

            // Normalize the RGB values
            std::transform(std::begin(rgb), std::end(rgb), std::begin(rgb), [](float c)
                           { return c * RGB_SCALE_FACTOR; });
            clr.emplace_back(rgb[0]);
            clr.emplace_back(rgb[1]);
            clr.emplace_back(rgb[2]);
        }
    }
    const long ptsCount = static_cast<int>(pts.size() / 3);

    // Create Open3D GPU tensors using data from host
    open3d::core::Tensor t_points(pts, {ptsCount, 3}, open3d::core::Dtype::Float32, open3d::core::Device("CUDA:0"));
    open3d::core::Tensor t_colors(clr, {ptsCount, 3}, open3d::core::Dtype::Float32, open3d::core::Device("CUDA:0"));

    // Build Open3D geometry
    open3d::t::geometry::PointCloud o3d_pc(open3d::core::Device("CUDA:0"));
    o3d_pc.SetPointPositions(t_points);
    o3d_pc.SetPointColors(t_colors);

    return o3d_pc;
}

inline sl::Mat Open3D_to_slMat(const open3d::geometry::PointCloud &pcd)
{
    const int num_points = static_cast<int>(pcd.points_.size());
    sl::Mat mat(num_points, 1, sl::MAT_TYPE::F32_C4);

    for (int i = 0; i < num_points; ++i)
    {
        // Copy point coordinates
        const auto &point = pcd.points_[i];
        // Copy color values
        const auto &color = pcd.colors_[i];
        uchar rgba[4];
        rgba[0] = static_cast<uchar>(color.x() * 255.0f);
        rgba[1] = static_cast<uchar>(color.y() * 255.0f);
        rgba[2] = static_cast<uchar>(color.z() * 255.0f);
        rgba[3] = static_cast<uchar>(1.0f * 255.0f); // alpha
        float packed_color = *reinterpret_cast<float *>(rgba);

        mat.setValue<sl::float4>(i, 0, sl::float4(point.x(), point.y(), point.z(), packed_color));
    }
    return mat;
}

inline sl::Mat Open3D_to_slMat_gpu(const open3d::t::geometry::PointCloud &pcd)
{
    auto pcd_cpu = pcd.To(open3d::core::Device("CPU:0"));
    auto pts = pcd.GetPointPositions().ToFlatVector<float>();
    auto clr = pcd.GetPointColors().ToFlatVector<float>();
    const int ptsCount = static_cast<int>(pts.size() / 3);

    sl::Mat mat(ptsCount, 1, sl::MAT_TYPE::F32_C4);
    for (int i = 0; i < pts.size(); i += 3)
    {
        uchar rgba[4];
        rgba[0] = static_cast<uchar>(clr[i] * 255.0f);
        rgba[1] = static_cast<uchar>(clr[i + 1] * 255.0f);
        rgba[2] = static_cast<uchar>(clr[i + 2] * 255.0f);
        rgba[3] = static_cast<uchar>(1.0f * 255.0f); // alpha
        float packed_color = *reinterpret_cast<float *>(rgba);
        mat.setValue<sl::float4>(i / 3, 0, sl::float4(pts[i], pts[i + 1], pts[i + 2], packed_color));
    }
    return mat;
}

#endif