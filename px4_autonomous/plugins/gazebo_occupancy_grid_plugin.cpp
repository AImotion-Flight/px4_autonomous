// Copyright 2018 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gazebo/physics/World.hh>
#include <plugins/gazebo_occupancy_grid_plugin.h>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_srvs/srv/empty.hpp>
#include <ignition/math/Vector3.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/RayShape.hh>

#include <memory>

namespace gazebo
{
/// Class to hold private data members (PIMPL pattern)
class OccupancyGridPluginPrivate
{
public:
  /// Connection to world update event. Callback is called while this is alive.
  gazebo::event::ConnectionPtr update_connection_;

  /// Node for ROS communication.
  gazebo_ros::Node::SharedPtr ros_node_;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr occupancy_grid_srv;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_pub;

  uint64_t width;
  uint64_t height;
  double step;
  uint64_t initial_x;
  uint64_t initial_y;
  uint64_t z;

  bool occupied(ignition::math::Vector3<double> cell, gazebo::physics::RayShapePtr ray) {
    if (cell.X() == this->initial_x && cell.Y() == this->initial_y) {
      return false;
    }

    ignition::math::Vector3 start_point = cell;
    ignition::math::Vector3 end_point = cell;

    double dist;
    std::string entity_name;

    end_point.X(end_point.X() + this->step);
    end_point.Y(end_point.Y() + this->step);

    ray->SetPoints(start_point, end_point);
    ray->GetIntersection(dist, entity_name);
    if (dist <= this->step) {
      return true;
    }

    start_point = cell;
    end_point = cell;
    start_point.X(start_point.X() + this->step);
    end_point.Y(end_point.Y() + this->step);

    ray->SetPoints(start_point, end_point);
    ray->GetIntersection(dist, entity_name);
    if (dist <= this->step) {
      return true;
    }

    return false;
  }
};

OccupancyGridPlugin::OccupancyGridPlugin()
: impl_(std::make_unique<OccupancyGridPluginPrivate>())
{
}

OccupancyGridPlugin::~OccupancyGridPlugin()
{
}

void OccupancyGridPlugin::Load(gazebo::physics::WorldPtr world, sdf::ElementPtr sdf)
{
  // Create a GazeboRos node instead of a common ROS node.
  // Pass it SDF parameters so common options like namespace and remapping
  // can be handled.
  impl_->ros_node_ = gazebo_ros::Node::Get(sdf);
  impl_->occupancy_grid_pub =
    impl_->ros_node_->create_publisher<nav_msgs::msg::OccupancyGrid>("occupancy_grid", 10);
  impl_->occupancy_grid_srv =
    impl_->ros_node_->create_service<std_srvs::srv::Empty>(
      "occupancy_grid_from_world",
      [this, world, sdf](const std::shared_ptr<std_srvs::srv::Empty::Request> request, std::shared_ptr<std_srvs::srv::Empty::Response> response) {
        nav_msgs::msg::OccupancyGrid grid;
        grid.header.stamp = impl_->ros_node_->get_clock()->now();
        grid.header.frame_id = "gazebo";

        grid.info.map_load_time = rclcpp::Time(0);
        grid.info.resolution = impl_->step;
        grid.info.width = impl_->width;
        grid.info.height = impl_->height;
        grid.info.origin.position.x = 0;
        grid.info.origin.position.y = 0;
        grid.info.origin.position.y = impl_->z;
        grid.info.origin.orientation.w = 1;

        gazebo::physics::PhysicsEnginePtr engine = world->Physics();
        engine->InitForThread();
        gazebo::physics::RayShapePtr ray =
          boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
            engine->CreateShape("ray", gazebo::physics::CollisionPtr()));

        RCLCPP_INFO(impl_->ros_node_->get_logger(), "Rasterizing world and checking collisions");

        grid.data.resize(impl_->width * impl_->height);
        for (int k = impl_->height - 1; k >= 0; k--)
        {
          for (size_t l = 0; l < impl_->width; l++)
          {
            ignition::math::Vector3<double> cell(l * impl_->step, k * impl_->step, impl_->z);
            if (impl_->occupied(cell, ray)) {
              grid.data[(impl_->height - 1 - k) * impl_->width + l] = 100;
            } else {
              grid.data[(impl_->height - 1 - k) * impl_->width + l] = 0;
            }
          }
        }

        impl_->occupancy_grid_pub->publish(grid);
      }
    );

  // The model pointer gives you direct access to the physics object,
  // for example:
  RCLCPP_INFO(impl_->ros_node_->get_logger(), world->Name().c_str());

  impl_->width = 10;
  if (sdf->HasElement("width"))
  {
    impl_->width = sdf->GetElement("width")->Get<uint64_t>();
  }

  impl_->height = 10;
  if (sdf->HasElement("height"))
  {
    impl_->height = sdf->GetElement("height")->Get<uint64_t>();
  }

  impl_->step = 1.0;
  if (sdf->HasElement("step"))
  {
    impl_->step = sdf->GetElement("step")->Get<double>();
  }

  impl_->initial_x = 0;
  if (sdf->HasElement("initial_x"))
  {
    impl_->initial_x = sdf->GetElement("initial_x")->Get<uint64_t>();
  }

  impl_->initial_y = 0;
  if (sdf->HasElement("initial_y"))
  {
    impl_->initial_y = sdf->GetElement("initial_y")->Get<uint64_t>();
  }

  impl_->z = 0.5;
  if (sdf->HasElement("z"))
  {
    impl_->z = sdf->GetElement("z")->Get<uint64_t>();
  }
}

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(OccupancyGridPlugin)
}  // namespace gazebo