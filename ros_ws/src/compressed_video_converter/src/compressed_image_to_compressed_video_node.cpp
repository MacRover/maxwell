// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2023-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: Apache-2.0
#include "compressed_video_converter/compressed_image_to_compressed_video_node.hpp"

namespace compressed_video_converter
{

CompressedImageToCompressedVideoNode::CompressedImageToCompressedVideoNode(
  rclcpp::NodeOptions options)
: Node("compressed_image_to_compressed_video",
    options.use_intra_process_comms(true)),
  compressed_video_pub_{
    create_publisher<foxglove_msgs::msg::CompressedVideo>(
      "video_compressed", rclcpp::QoS(10))},
  compressed_image_sub_{
    create_subscription<sensor_msgs::msg::CompressedImage>(
      "image_compressed", 10,
      [this](const sensor_msgs::msg::CompressedImage::SharedPtr compressed_img_msg) {
        auto compressed_video_msg = foxglove_msgs::msg::CompressedVideo();
        compressed_video_msg.timestamp =
        compressed_img_msg->header.stamp;
        compressed_video_msg.frame_id =
        compressed_img_msg->header.frame_id;
        compressed_video_msg.data = compressed_img_msg->data;
        compressed_video_msg.format = compressed_img_msg->format;
        compressed_video_pub_->publish(compressed_video_msg);
      })} {}

}  // namespace compressed_video_converter

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CompressedImageToCompressedVideoNode>());
  rclcpp::shutdown();
  
  return 0;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(
  compressed_video_converter::CompressedImageToCompressedVideoNode)