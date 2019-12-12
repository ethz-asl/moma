#include <sstream>
#include <string>
#include "std_msgs/Empty.h"
#include "semantic_button_panel.h"

namespace control_gui
{

SemanticButtonPanel::SemanticButtonPanel(QWidget *parent) : rviz::Panel(parent), node_handle("manipulation_actions")
{
    reset_publisher = node_handle.advertise<std_msgs::Empty>("reset", 1000);
    instance_subscriber = node_handle.subscribe("/grasp_selection/instances", 1, &SemanticButtonPanel::instancesCallback, this);

    // Setup Panel.
    layout = new QVBoxLayout;
    scan_button = new QPushButton("Scan scene and plan grasp");
    grasp_button = new QPushButton("Grasp object");
    stow_button = new QPushButton("Stow object");
    reset_button = new QPushButton("Reset object");
    layout->addWidget(scan_button);
    layout->addWidget(grasp_button);
    layout->addWidget(stow_button);
    layout->addWidget(reset_button);
    setLayout(layout);

    connect(reset_button, &QPushButton::clicked, this, &SemanticButtonPanel::onResetButtonClicked);
    setButtons();
}

void SemanticButtonPanel::onResetButtonClicked()
{
    ROS_INFO("Reset object button was clicked");
    std_msgs::Empty msg;
    reset_publisher.publish(msg);
}

void SemanticButtonPanel::instancesCallback(const grasp_demo::Instances &msg) {
    layout->removeWidget(grasp_button);
    layout->removeWidget(stow_button);
    layout->removeWidget(reset_button);

    std::vector<const char*> labels = {"one", "two", "three" };
    for (int i = 0; i < 3; i++) {
        QPushButton *button = new QPushButton(labels[i]);
        layout->addWidget(button);
    }
    layout->addWidget(stow_button);
    layout->addWidget(reset_button);
}

void SemanticButtonPanel::setButtons() {
    layout->removeWidget(grasp_button);
    layout->removeWidget(stow_button);
    layout->removeWidget(reset_button);
    while (!instance_buttons.empty()) {
        QPushButton *button = instance_buttons.back();
        instance_buttons.pop_back();
        layout->removeWidget(button);
        disconnect(button, &QPushButton::clicked, this, &SemanticButtonPanel::onInstanceGraspButtonClicked);
        delete button;
    }

    std::vector<const char*> labels = {"one", "two", "three" };
    for (int i = 0; i < 3; i++) {
        //std::sstream stream;
        //stream << "Grasp " << labels[i];
        QPushButton *button = new QPushButton(labels[i]);
        instance_buttons.push_back(button);
        layout->addWidget(button);
        connect(button, &QPushButton::clicked, this, &SemanticButtonPanel::onInstanceGraspButtonClicked);
    }
    layout->addWidget(stow_button);
    layout->addWidget(reset_button);
}

void SemanticButtonPanel::onInstanceGraspButtonClicked() {
    ROS_INFO("button clicked");
    setButtons();
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(control_gui::SemanticButtonPanel, rviz::Panel)

