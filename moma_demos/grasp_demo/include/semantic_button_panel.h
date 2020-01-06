#ifndef SEMANTIC_BUTTON_PANEL_H
#define SEMANTIC_BUTTON_PANEL_H

#include <ros/ros.h>
#include <rviz/panel.h>
#include <QVBoxLayout>
#include <QPushButton>
#include <QWidget>
#include "grasp_demo/Instances.h"

namespace control_gui
{

class InstancePushButton : public QPushButton {
    private:
    int instance_id;
    ros::Publisher *topic_pub;

    public:
    InstancePushButton(const char *label, int instance_id, ros::Publisher *topic_pub);
    ~InstancePushButton();
    public Q_SLOTS:
    void onButtonClick();
};

class SemanticButtonPanel : public rviz::Panel {
    public:
    SemanticButtonPanel(QWidget *parent = 0);
    ~SemanticButtonPanel();

    public Q_SLOTS:
    void onScanButtonClicked();
    void onStowButtonClicked();
    void onResetButtonClicked();
    void instancesCallback(const grasp_demo::Instances &msg);

    private:
    ros::NodeHandle node_handle;
    ros::Publisher scan_publisher, stow_publisher, reset_publisher;
    ros::Subscriber instance_subscriber;
    ros::Publisher instance_publisher;
    QVBoxLayout *layout;
    QPushButton *scan_button;
    QPushButton *stow_button;
    QPushButton *reset_button;
    std::vector<InstancePushButton*> instance_buttons;
};
}
#endif // SEMANTIC_BUTTON_PANEL_H
