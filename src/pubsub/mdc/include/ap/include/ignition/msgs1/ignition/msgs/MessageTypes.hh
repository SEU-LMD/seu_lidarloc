/*
 * Copyright (C) 2017 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

/* This file was automatically generated via CMake.
 * Do not edit this directly, instead see:
 *      ign-msgs/src/MessageTypes.hh.in
 */

#ifndef IGNITION_MSGS_MESSAGE_TYPES_HH_
#define IGNITION_MSGS_MESSAGE_TYPES_HH_

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable: 4100 4512 4127 4068 4244 4267 4251 4146)
#endif

#include <ignition/msgs/altimeter.pb.h>
#include <ignition/msgs/any.pb.h>
#include <ignition/msgs/atmosphere.pb.h>
#include <ignition/msgs/axis.pb.h>
#include <ignition/msgs/battery.pb.h>
#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/boxgeom.pb.h>
#include <ignition/msgs/bytes.pb.h>
#include <ignition/msgs/camera_cmd.pb.h>
#include <ignition/msgs/camera_lens.pb.h>
#include <ignition/msgs/camerasensor.pb.h>
#include <ignition/msgs/cessna.pb.h>
#include <ignition/msgs/clock.pb.h>
#include <ignition/msgs/cmd_vel2d.pb.h>
#include <ignition/msgs/collision.pb.h>
#include <ignition/msgs/color.pb.h>
#include <ignition/msgs/contact.pb.h>
#include <ignition/msgs/contacts.pb.h>
#include <ignition/msgs/contactsensor.pb.h>
#include <ignition/msgs/cylindergeom.pb.h>
#include <ignition/msgs/density.pb.h>
#include <ignition/msgs/diagnostics.pb.h>
#include <ignition/msgs/distortion.pb.h>
#include <ignition/msgs/double.pb.h>
#include <ignition/msgs/double_v.pb.h>
#include <ignition/msgs/duration.pb.h>
#include <ignition/msgs/empty.pb.h>
#include <ignition/msgs/entity_factory.pb.h>
#include <ignition/msgs/float.pb.h>
#include <ignition/msgs/float_v.pb.h>
#include <ignition/msgs/fluid.pb.h>
#include <ignition/msgs/fog.pb.h>
#include <ignition/msgs/friction.pb.h>
#include <ignition/msgs/geometry.pb.h>
#include <ignition/msgs/gps.pb.h>
#include <ignition/msgs/gps_sensor.pb.h>
#include <ignition/msgs/gui.pb.h>
#include <ignition/msgs/gui_camera.pb.h>
#include <ignition/msgs/header.pb.h>
#include <ignition/msgs/heightmapgeom.pb.h>
#include <ignition/msgs/hydra.pb.h>
#include <ignition/msgs/image.pb.h>
#include <ignition/msgs/imagegeom.pb.h>
#include <ignition/msgs/imu.pb.h>
#include <ignition/msgs/imu_sensor.pb.h>
#include <ignition/msgs/inertial.pb.h>
#include <ignition/msgs/int32.pb.h>
#include <ignition/msgs/int32_v.pb.h>
#include <ignition/msgs/int64.pb.h>
#include <ignition/msgs/int64_v.pb.h>
#include <ignition/msgs/joint.pb.h>
#include <ignition/msgs/joint_animation.pb.h>
#include <ignition/msgs/joint_cmd.pb.h>
#include <ignition/msgs/joint_wrench.pb.h>
#include <ignition/msgs/joy.pb.h>
#include <ignition/msgs/joystick.pb.h>
#include <ignition/msgs/laserscan.pb.h>
#include <ignition/msgs/light.pb.h>
#include <ignition/msgs/link.pb.h>
#include <ignition/msgs/link_data.pb.h>
#include <ignition/msgs/log_control.pb.h>
#include <ignition/msgs/log_playback_control.pb.h>
#include <ignition/msgs/log_playback_stats.pb.h>
#include <ignition/msgs/log_status.pb.h>
#include <ignition/msgs/logical_camera_image.pb.h>
#include <ignition/msgs/logical_camera_sensor.pb.h>
#include <ignition/msgs/magnetometer.pb.h>
#include <ignition/msgs/marker.pb.h>
#include <ignition/msgs/marker_v.pb.h>
#include <ignition/msgs/material.pb.h>
#include <ignition/msgs/meshgeom.pb.h>
#include <ignition/msgs/model.pb.h>
#include <ignition/msgs/model_configuration.pb.h>
#include <ignition/msgs/model_v.pb.h>
#include <ignition/msgs/packet.pb.h>
#include <ignition/msgs/param.pb.h>
#include <ignition/msgs/param_v.pb.h>
#include <ignition/msgs/physics.pb.h>
#include <ignition/msgs/pid.pb.h>
#include <ignition/msgs/planegeom.pb.h>
#include <ignition/msgs/plugin.pb.h>
#include <ignition/msgs/plugin_v.pb.h>
#include <ignition/msgs/pointcloud.pb.h>
#include <ignition/msgs/polylinegeom.pb.h>
#include <ignition/msgs/pose.pb.h>
#include <ignition/msgs/pose_animation.pb.h>
#include <ignition/msgs/pose_trajectory.pb.h>
#include <ignition/msgs/pose_v.pb.h>
#include <ignition/msgs/projector.pb.h>
#include <ignition/msgs/propagation_grid.pb.h>
#include <ignition/msgs/propagation_particle.pb.h>
#include <ignition/msgs/publish.pb.h>
#include <ignition/msgs/publishers.pb.h>
#include <ignition/msgs/quaternion.pb.h>
#include <ignition/msgs/raysensor.pb.h>
#include <ignition/msgs/request.pb.h>
#include <ignition/msgs/response.pb.h>
#include <ignition/msgs/rest_login.pb.h>
#include <ignition/msgs/rest_logout.pb.h>
#include <ignition/msgs/rest_post.pb.h>
#include <ignition/msgs/rest_response.pb.h>
#include <ignition/msgs/road.pb.h>
#include <ignition/msgs/scene.pb.h>
#include <ignition/msgs/selection.pb.h>
#include <ignition/msgs/sensor.pb.h>
#include <ignition/msgs/sensor_noise.pb.h>
#include <ignition/msgs/sensor_v.pb.h>
#include <ignition/msgs/server_control.pb.h>
#include <ignition/msgs/shadows.pb.h>
#include <ignition/msgs/sim_event.pb.h>
#include <ignition/msgs/sky.pb.h>
#include <ignition/msgs/sonar.pb.h>
#include <ignition/msgs/spheregeom.pb.h>
#include <ignition/msgs/spherical_coordinates.pb.h>
#include <ignition/msgs/stringmsg.pb.h>
#include <ignition/msgs/stringmsg_v.pb.h>
#include <ignition/msgs/subscribe.pb.h>
#include <ignition/msgs/surface.pb.h>
#include <ignition/msgs/tactile.pb.h>
#include <ignition/msgs/test.pb.h>
#include <ignition/msgs/time.pb.h>
#include <ignition/msgs/topic_info.pb.h>
#include <ignition/msgs/track_visual.pb.h>
#include <ignition/msgs/uint32.pb.h>
#include <ignition/msgs/uint32_v.pb.h>
#include <ignition/msgs/uint64.pb.h>
#include <ignition/msgs/uint64_v.pb.h>
#include <ignition/msgs/undo_redo.pb.h>
#include <ignition/msgs/user_cmd.pb.h>
#include <ignition/msgs/user_cmd_stats.pb.h>
#include <ignition/msgs/vector2d.pb.h>
#include <ignition/msgs/vector3d.pb.h>
#include <ignition/msgs/visual.pb.h>
#include <ignition/msgs/visual_v.pb.h>
#include <ignition/msgs/web_request.pb.h>
#include <ignition/msgs/wind.pb.h>
#include <ignition/msgs/wireless_node.pb.h>
#include <ignition/msgs/wireless_nodes.pb.h>
#include <ignition/msgs/world_control.pb.h>
#include <ignition/msgs/world_modify.pb.h>
#include <ignition/msgs/world_reset.pb.h>
#include <ignition/msgs/world_stats.pb.h>
#include <ignition/msgs/wrench.pb.h>



#ifdef _MSC_VER
#pragma warning(pop)
#endif

#endif
