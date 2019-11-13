#include "std_msgs/Empty.h"
#include "button_panel.h"
// #include "panda_grasp_demo/ButtonPress.h"

namespace control_gui {

ButtonPanel::ButtonPanel(QWidget *parent) : rviz::Panel(parent), node_handle("manipulation_actions") {
    scan_publisher = node_handle.advertise<std_msgs::Empty>("scan", 1000);
    grasp_publisher = node_handle.advertise<std_msgs::Empty>("grasp", 1000);
    stow_publisher = node_handle.advertise<std_msgs::Empty>("stow", 1000);

    // Setup Panel.
    QVBoxLayout *layout = new QVBoxLayout;
    QPushButton *scan_button = new QPushButton("Scan scene and plan grasp");
    QPushButton *grasp_button = new QPushButton("Grasp object");
    QPushButton *stow_button = new QPushButton("Stow object");
    layout->addWidget(scan_button);
    layout->addWidget(grasp_button);
    layout->addWidget(stow_button);
    setLayout(layout);

    connect(scan_button, &QPushButton::clicked, this, &ButtonPanel::onScanButtonClicked);
    connect(grasp_button, &QPushButton::clicked, this, &ButtonPanel::onGraspButtonClicked);
    connect(stow_button, &QPushButton::clicked, this, &ButtonPanel::onStowButtonClicked);
}

void ButtonPanel::onScanButtonClicked() {
    ROS_INFO("Scan button was clicked.");
    std_msgs::Empty msg;
    scan_publisher.publish(msg);
}

void ButtonPanel::onGraspButtonClicked() {
    ROS_INFO("Grasp button was clicked.");
    std_msgs::Empty msg;
    grasp_publisher.publish(msg);
}

void ButtonPanel::onStowButtonClicked() {
    ROS_INFO("Drop object button was clicked.");
    std_msgs::Empty msg;
    stow_publisher.publish(msg);
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(control_gui::ButtonPanel, rviz::Panel)

