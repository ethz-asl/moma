#include <sstream>
#include <string>
#include "std_msgs/Empty.h"
#include "semantic_button_panel.h"
#include "grasp_demo/Instance.h"
namespace control_gui { InstancePushButton::InstancePushButton(const char *label, int id, ros::Publisher *publisher) : QPushButton(label) {
    instance_id = id;
    topic_pub = publisher;
    connect(this, &InstancePushButton::clicked, this, &InstancePushButton::onButtonClick);
}

InstancePushButton::~InstancePushButton() {
    disconnect(this, &InstancePushButton::clicked, this, &InstancePushButton::onButtonClick);
}

void InstancePushButton::onButtonClick() {
    ROS_INFO_STREAM("Instance " << instance_id << " button clicked");
    grasp_demo::Instance instance_msg;
    instance_msg.id = instance_id;
    topic_pub->publish(instance_msg);
}

SemanticButtonPanel::SemanticButtonPanel(QWidget *parent) : rviz::Panel(parent), node_handle("manipulation_actions")
{
    scan_publisher = node_handle.advertise<std_msgs::Empty>("scan", 1);
    stow_publisher = node_handle.advertise<std_msgs::Empty>("stow", 1);
    reset_publisher = node_handle.advertise<std_msgs::Empty>("reset", 1);
    instance_subscriber = node_handle.subscribe("/commander_node/instances", 1, &SemanticButtonPanel::instancesCallback, this);

    instance_publisher = node_handle.advertise<grasp_demo::Instance>("selected_instance", 1);

    // Setup Panel.
    layout = new QVBoxLayout;
    scan_button = new QPushButton("Scan scene and plan grasp");
    stow_button = new QPushButton("Stow object");
    reset_button = new QPushButton("Reset object");
    layout->addWidget(scan_button);
    layout->addWidget(stow_button);
    layout->addWidget(reset_button);
    setLayout(layout);

    for (int i=0; i < 2; i++) {
        //int instance_id = msg.instance_ids[i];
        //std::string category = msg.semantic_categories[i];
        std::string category = "Book";
        int instance_id = i;
        std::stringstream label_stream;
        label_stream << "Grasp " << category;
        std::string label = label_stream.str();

        ROS_INFO_STREAM("" << label.c_str());
        InstancePushButton *button = new InstancePushButton(label.c_str(), instance_id, &instance_publisher);
        layout->addWidget(button);
        instance_buttons.push_back(button);
    }

    connect(scan_button, &QPushButton::clicked, this, &SemanticButtonPanel::onScanButtonClicked);
    connect(stow_button, &QPushButton::clicked, this, &SemanticButtonPanel::onStowButtonClicked);
    connect(reset_button, &QPushButton::clicked, this, &SemanticButtonPanel::onResetButtonClicked);
}

SemanticButtonPanel::~SemanticButtonPanel() {
    for (InstancePushButton *button : instance_buttons) {
        delete button;
    }
}

void SemanticButtonPanel::onScanButtonClicked() {
    ROS_INFO("Scan button was clicked");
    std_msgs::Empty msg;
    scan_publisher.publish(msg);
}

void SemanticButtonPanel::onStowButtonClicked() {
    ROS_INFO("Stow button was clicked");
    std_msgs::Empty msg;
    stow_publisher.publish(msg);
}

void SemanticButtonPanel::onResetButtonClicked() {
    ROS_INFO("Reset object button was clicked");
    std_msgs::Empty msg;
    reset_publisher.publish(msg);
}

void SemanticButtonPanel::instancesCallback(const grasp_demo::Instances &msg) {
    layout->removeWidget(stow_button);
    layout->removeWidget(reset_button);

    while (!instance_buttons.empty()) {
        InstancePushButton *button = instance_buttons.back();
        instance_buttons.pop_back();
        layout->removeWidget(button);
        delete button;
    }

    for (int i=0; i < msg.instance_ids.size(); i++) {
        int instance_id = msg.instance_ids[i];
        std::string category = msg.semantic_categories[i];

        std::stringstream label_stream;
        label_stream << "Grasp " << category;
        std::string label = label_stream.str();

        InstancePushButton *button = new InstancePushButton(label.c_str(), instance_id, &instance_publisher);
        layout->addWidget(button);
        instance_buttons.push_back(button);
    }
    layout->addWidget(stow_button);
    layout->addWidget(reset_button);
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(control_gui::SemanticButtonPanel, rviz::Panel)

