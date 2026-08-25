#pragma once

#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <cilantro/utilities/point_cloud.hpp>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <geometry_msgs/msg/transform.hpp>
#include <limits>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/header.hpp>
#include <stdexcept>
#include <string>

namespace dynamic_obstacle_tracker {

inline std::string normalizeFrame(std::string frame)
{
    while (!frame.empty() && frame.front() == '/') frame.erase(frame.begin());
    return frame;
}

namespace detail {

inline bool nativeIsBigEndian()
{
    const std::uint16_t value = 0x0102;
    return *reinterpret_cast<const std::uint8_t*>(&value) == 0x01;
}

inline std::size_t pointFieldSize(std::uint8_t datatype)
{
    switch (datatype) {
        case sensor_msgs::msg::PointField::INT8:
        case sensor_msgs::msg::PointField::UINT8: return 1;
        case sensor_msgs::msg::PointField::INT16:
        case sensor_msgs::msg::PointField::UINT16: return 2;
        case sensor_msgs::msg::PointField::INT32:
        case sensor_msgs::msg::PointField::UINT32:
        case sensor_msgs::msg::PointField::FLOAT32: return 4;
        case sensor_msgs::msg::PointField::FLOAT64: return 8;
        default: throw std::invalid_argument("PointCloud2 contains an unsupported XYZ datatype");
    }
}

template <typename Scalar>
Scalar readScalar(const std::uint8_t* data, bool swap_bytes)
{
    if (!swap_bytes) {
        Scalar value;
        std::memcpy(&value, data, sizeof(Scalar));
        return value;
    }

    std::array<std::uint8_t, sizeof(Scalar)> bytes;
    std::copy_n(data, sizeof(Scalar), bytes.begin());
    std::reverse(bytes.begin(), bytes.end());

    Scalar value;
    std::memcpy(&value, bytes.data(), sizeof(Scalar));
    return value;
}

inline float readPointField(const std::uint8_t* point_data, const sensor_msgs::msg::PointField& field, bool swap_bytes)
{
    const std::uint8_t* field_data = point_data + field.offset;
    switch (field.datatype) {
        case sensor_msgs::msg::PointField::INT8: return static_cast<float>(readScalar<std::int8_t>(field_data, false));
        case sensor_msgs::msg::PointField::UINT8:
            return static_cast<float>(readScalar<std::uint8_t>(field_data, false));
        case sensor_msgs::msg::PointField::INT16:
            return static_cast<float>(readScalar<std::int16_t>(field_data, swap_bytes));
        case sensor_msgs::msg::PointField::UINT16:
            return static_cast<float>(readScalar<std::uint16_t>(field_data, swap_bytes));
        case sensor_msgs::msg::PointField::INT32:
            return static_cast<float>(readScalar<std::int32_t>(field_data, swap_bytes));
        case sensor_msgs::msg::PointField::UINT32:
            return static_cast<float>(readScalar<std::uint32_t>(field_data, swap_bytes));
        case sensor_msgs::msg::PointField::FLOAT32: return readScalar<float>(field_data, swap_bytes);
        case sensor_msgs::msg::PointField::FLOAT64:
            return static_cast<float>(readScalar<double>(field_data, swap_bytes));
        default: throw std::invalid_argument("PointCloud2 contains an unsupported XYZ datatype");
    }
}

inline const sensor_msgs::msg::PointField& findPointField(
        const sensor_msgs::msg::PointCloud2& cloud,
        const std::string&                   name)
{
    const auto field = std::find_if(
            cloud.fields.begin(), cloud.fields.end(), [&](const auto& candidate) { return candidate.name == name; });
    if (field == cloud.fields.end())
        throw std::invalid_argument("PointCloud2 is missing the '" + name + "' field");
    if (field->count < 1)
        throw std::invalid_argument("PointCloud2 field '" + name + "' is empty");
    if (static_cast<std::size_t>(field->offset) + pointFieldSize(field->datatype) > cloud.point_step)
        throw std::invalid_argument("PointCloud2 field '" + name + "' exceeds point_step");
    return *field;
}

} // namespace detail

inline cilantro::PointCloud3f pointCloud2ToCilantro(const sensor_msgs::msg::PointCloud2& message)
{
    cilantro::PointCloud3f cloud;
    if (message.width == 0 || message.height == 0)
        return cloud;
    if (message.point_step == 0)
        throw std::invalid_argument("PointCloud2 point_step must be positive");

    const auto& x_field = detail::findPointField(message, "x");
    const auto& y_field = detail::findPointField(message, "y");
    const auto& z_field = detail::findPointField(message, "z");

    const std::size_t width  = message.width;
    const std::size_t height = message.height;
    if (width > std::numeric_limits<std::size_t>::max() / height)
        throw std::invalid_argument("PointCloud2 dimensions overflow the host size type");
    const std::size_t point_count = width * height;
    if (point_count > static_cast<std::size_t>(std::numeric_limits<Eigen::Index>::max()))
        throw std::invalid_argument("PointCloud2 contains too many points for Eigen storage");
    if (width > std::numeric_limits<std::size_t>::max() / message.point_step)
        throw std::invalid_argument("PointCloud2 row size overflows the host size type");
    const std::size_t packed_row_size = width * message.point_step;
    if (message.row_step < packed_row_size)
        throw std::invalid_argument("PointCloud2 row_step is smaller than its packed row size");
    if (height - 1 > (std::numeric_limits<std::size_t>::max() - packed_row_size) / message.row_step)
        throw std::invalid_argument("PointCloud2 data size overflows the host size type");
    const std::size_t required_size = (height - 1) * message.row_step + packed_row_size;
    if (message.data.size() < required_size)
        throw std::invalid_argument("PointCloud2 data is smaller than its declared dimensions");

    cloud.points.resize(3, static_cast<Eigen::Index>(point_count));
    const bool swap_bytes       = message.is_bigendian != detail::nativeIsBigEndian();
    const bool native_float_xyz = !swap_bytes && x_field.datatype == sensor_msgs::msg::PointField::FLOAT32
                               && y_field.datatype == sensor_msgs::msg::PointField::FLOAT32
                               && z_field.datatype == sensor_msgs::msg::PointField::FLOAT32
                               && y_field.offset == x_field.offset + sizeof(float)
                               && z_field.offset == y_field.offset + sizeof(float);
    if (native_float_xyz && x_field.offset == 0 && message.point_step == 3 * sizeof(float)
        && message.row_step == packed_row_size) {
        std::memcpy(cloud.points.data(), message.data.data(), point_count * 3 * sizeof(float));
        return cloud;
    }

    Eigen::Index output_index = 0;
    for (std::size_t row = 0; row < height; ++row) {
        const std::uint8_t* point_data = message.data.data() + row * message.row_step;
        for (std::size_t column = 0; column < width; ++column) {
            if (native_float_xyz) {
                std::memcpy(cloud.points.col(output_index++).data(), point_data + x_field.offset, 3 * sizeof(float));
            } else {
                cloud.points.col(output_index++) = Eigen::Vector3f(
                        detail::readPointField(point_data, x_field, swap_bytes),
                        detail::readPointField(point_data, y_field, swap_bytes),
                        detail::readPointField(point_data, z_field, swap_bytes));
            }
            point_data += message.point_step;
        }
    }
    return cloud;
}

inline sensor_msgs::msg::PointCloud2 cilantroToPointCloud2(
        const cilantro::PointCloud3f& cloud,
        const std_msgs::msg::Header&  header)
{
    const std::size_t     point_count = cloud.size();
    constexpr std::size_t kPointStep  = 3 * sizeof(float);
    if (point_count > std::numeric_limits<std::uint32_t>::max() / kPointStep)
        throw std::invalid_argument("cilantro point cloud is too large for PointCloud2");

    sensor_msgs::msg::PointCloud2 message;
    message.header = header;
    message.height = 1;
    message.width  = static_cast<std::uint32_t>(point_count);
    message.fields.resize(3);
    for (std::size_t index = 0; index < message.fields.size(); ++index) {
        message.fields[index].name     = std::string(1, static_cast<char>('x' + index));
        message.fields[index].offset   = static_cast<std::uint32_t>(index * sizeof(float));
        message.fields[index].datatype = sensor_msgs::msg::PointField::FLOAT32;
        message.fields[index].count    = 1;
    }
    message.is_bigendian = detail::nativeIsBigEndian();
    message.point_step   = kPointStep;
    message.row_step     = message.width * message.point_step;
    message.is_dense     = cloud.points.allFinite();
    message.data.resize(static_cast<std::size_t>(message.row_step));

    if (!message.data.empty())
        std::memcpy(message.data.data(), cloud.points.data(), message.data.size());
    return message;
}

inline Eigen::Isometry3f transformToEigen(const geometry_msgs::msg::Transform& transform)
{
    const auto&           rotation = transform.rotation;
    Eigen::Quaternionf    quaternion(rotation.w, rotation.x, rotation.y, rotation.z);
    const Eigen::Vector3f translation(transform.translation.x, transform.translation.y, transform.translation.z);
    if (!quaternion.coeffs().allFinite() || !translation.allFinite() || quaternion.norm() < 1e-6F)
        throw std::invalid_argument("TF transform contains invalid values");

    quaternion.normalize();
    Eigen::Isometry3f result = Eigen::Isometry3f::Identity();
    result.linear()          = quaternion.toRotationMatrix();
    result.translation()     = translation;
    return result;
}

inline void transformPointCloud(cilantro::PointCloud3f& cloud, const geometry_msgs::msg::Transform& transform)
{
    cloud.transform(transformToEigen(transform));
}

} // namespace dynamic_obstacle_tracker
