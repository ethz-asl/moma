#ifndef BUTTON_PANEL_H
#define BUTTON_PANEL_H

#include <ros/ros.h>
#include <rviz/panel.h>
#include <QVBoxLayout>
#include <QPushButton>
#include <QWidget>
#include "grasp_demo/Instances.h"

namespace control_gui {

class ButtonPanel : public rviz::Panel {
    public:
    ButtonPanel(QWidget *parent = 0);

    public Q_SLOTS:
    void onScanButtonClicked();
    void onGraspButtonClicked();
    void onStowButtonClicked();
    void onResetButtonClicked();
    void instancesCallback(const grasp_demo::Instances &msg);
    void setButtons();
    void onInstanceGraspButtonClicked();

    private:
    ros::NodeHandle node_handle;
    ros::Publisher scan_publisher, grasp_publisher, stow_publisher, reset_publisher;
    ros::Subscriber instance_subscriber;
    QVBoxLayout *layout;
    QPushButton *scan_button;
    QPushButton *grasp_button;
    QPushButton *stow_button;
    QPushButton *reset_button;
    std::vector<QPushButton*> instance_buttons;
};
}
#endif // BUTTON_PANEL_H

