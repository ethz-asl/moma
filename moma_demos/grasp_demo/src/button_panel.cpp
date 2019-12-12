#include "std_msgs/Empty.h"
#include "button_panel.h"

namespace control_gui
{

ButtonPanel::ButtonPanel(QWidget *parent) : rviz::Panel(parent), node_handle("manipulation_actions")
{
    scan_publisher = node_handle.advertise<std_msgs::Empty>("scan", 1000);
    grasp_publisher = node_handle.advertise<std_msgs::Empty>("grasp", 1000);
    stow_publisher = node_handle.advertise<std_msgs::Empty>("stow", 1000);
    reset_publisher = node_handle.advertise<std_msgs::Empty>("reset", 1000);
    next_publisher = node_handle.advertise<std_msgs::Empty>("next", 1000);
    repeat_publisher = node_handle.advertise<std_msgs::Empty>("repeat", 1000);

    // Setup Panel.
    QVBoxLayout *layout = new QVBoxLayout;
    QPushButton *scan_button = new QPushButton("Scan scene and plan grasp");
    QPushButton *grasp_button = new QPushButton("Grasp object");
    QPushButton *stow_button = new QPushButton("Stow object");
    QPushButton *reset_button = new QPushButton("Reset");
    QPushButton *next_button = new QPushButton("Next action");
    QPushButton *repeat_button = new QPushButton("Repeat last action");
    layout->addWidget(scan_button);
    layout->addWidget(grasp_button);
    layout->addWidget(stow_button);
    layout->addWidget(reset_button);
    layout->addWidget(next_button);
    layout->addWidget(repeat_button);
    setLayout(layout);

    connect(scan_button, &QPushButton::clicked, this, &ButtonPanel::onScanButtonClicked);
    connect(grasp_button, &QPushButton::clicked, this, &ButtonPanel::onGraspButtonClicked);
    connect(stow_button, &QPushButton::clicked, this, &ButtonPanel::onStowButtonClicked);
    connect(reset_button, &QPushButton::clicked, this, &ButtonPanel::onResetButtonClicked);
    connect(next_button, &QPushButton::clicked, this, &ButtonPanel::onNextButtonClicked);
    connect(repeat_button, &QPushButton::clicked, this, &ButtonPanel::onRepeatButtonClicked);
}

void ButtonPanel::onScanButtonClicked()
{
    ROS_INFO("Scan button was clicked.");
    std_msgs::Empty msg;
    scan_publisher.publish(msg);
}

void ButtonPanel::onGraspButtonClicked()
{
    ROS_INFO("Grasp button was clicked.");
    std_msgs::Empty msg;
    grasp_publisher.publish(msg);
}

void ButtonPanel::onStowButtonClicked()
{
    ROS_INFO("Drop object button was clicked.");
    std_msgs::Empty msg;
    stow_publisher.publish(msg);
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
