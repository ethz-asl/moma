#include <sstream>
#include <string>
#include "std_msgs/Empty.h"
#include "button_panel.h"

namespace control_gui
{

ButtonPanel::ButtonPanel(QWidget *parent) : rviz::Panel(parent), node_handle("manipulation_actions")
{
    reset_publisher = node_handle.advertise<std_msgs::Empty>("reset", 1000);
    next_publisher = node_handle.advertise<std_msgs::Empty>("next", 1000);
    repeat_publisher = node_handle.advertise<std_msgs::Empty>("repeat", 1000);

    // Setup Panel.
    QVBoxLayout *layout = new QVBoxLayout;
    QPushButton *next_button = new QPushButton("Next action");
    QPushButton *repeat_button = new QPushButton("Repeat last action");
    QPushButton *reset_button = new QPushButton("Reset");
    layout->addWidget(next_button);
    layout->addWidget(repeat_button);
    layout->addWidget(reset_button);
    setLayout(layout);

    connect(reset_button, &QPushButton::clicked, this, &ButtonPanel::onResetButtonClicked);
    connect(next_button, &QPushButton::clicked, this, &ButtonPanel::onNextButtonClicked);
    connect(repeat_button, &QPushButton::clicked, this, &ButtonPanel::onRepeatButtonClicked);
}

void ButtonPanel::onResetButtonClicked()
{
    ROS_INFO("Reset object button was clicked");
    std_msgs::Empty msg;
    reset_publisher.publish(msg);
}

void ButtonPanel::onNextButtonClicked()
{
    ROS_INFO("Next button was clicked");
    std_msgs::Empty msg;
    next_publisher.publish(msg);
}

void ButtonPanel::onRepeatButtonClicked()
{
    ROS_INFO("Repeat button was clicked");
    std_msgs::Empty msg;
    repeat_publisher.publish(msg);
}

} // namespace control_gui

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(control_gui::ButtonPanel, rviz::Panel)
