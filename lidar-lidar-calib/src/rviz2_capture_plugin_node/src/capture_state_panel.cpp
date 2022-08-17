// Copyright 2022 AutoCore
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// you may obtain a copy of the License at
// 
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "include/rviz2_capture_plugin_node/capture_state_panel.hpp"
#include <QString>
#include <QVBoxLayout>
#include <rviz_common/display_context.hpp>

#include <memory>

using std::placeholders::_1;

CaptureStatePanel::CaptureStatePanel(QWidget * parent) : rviz_common::Panel(parent)
{
    // Capture Engage Button
    capture_button_ptr_ = new QPushButton("Capture");
    connect(capture_button_ptr_, SIGNAL(clicked()), SLOT(onClickCaptureStatePanel()));

    auto * v_layout = new QVBoxLayout;
    v_layout->addWidget(capture_button_ptr_);
    setLayout(v_layout);
}

void CaptureStatePanel::onInitialize()
{
    raw_node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
    client_capture_ = raw_node_->create_client<rviz2_capture_plugin_interface::srv::Capture>(
        "/multi_lidar_calib/capture", rmw_qos_profile_services_default);
}

void CaptureStatePanel::onClickCaptureStatePanel()
{
    auto req = std::make_shared<rviz2_capture_plugin_interface::srv::Capture::Request>();
    req->capture = true;
    RCLCPP_INFO(raw_node_->get_logger(), "client request");

    if (!client_capture_->service_is_ready()) {
        RCLCPP_INFO(raw_node_->get_logger(), "server is not ready, request ignored.");
        return;
    }

    client_capture_->async_send_request(
    req, [this](rclcpp::Client<rviz2_capture_plugin_interface::srv::Capture>::SharedFuture result) {
      RCLCPP_INFO(
        raw_node_->get_logger(), "Status from server: %d", result.get()->status);
    });
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(CaptureStatePanel, rviz_common::Panel)
