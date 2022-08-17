#ifndef CAPTURE_STATE_PANEL_HPP_
#define CAPTURE_STATE_PANEL_HPP_

#include <QPushButton>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include "rviz2_capture_plugin_interface/srv/capture.hpp"

class CaptureStatePanel : public rviz_common::Panel
{

    Q_OBJECT

public:
    explicit CaptureStatePanel(QWidget * parent = nullptr);
    void onInitialize() override;

public Q_SLOTS:
    void onClickCaptureStatePanel();

protected:
    rclcpp::Node::SharedPtr raw_node_;
    rclcpp::Client<rviz2_capture_plugin_interface::srv::Capture>::SharedPtr client_capture_;
    QPushButton * capture_button_ptr_;
};

#endif //CAPTURE_STATE_PANEL_HPP_