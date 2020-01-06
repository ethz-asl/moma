#ifndef BUTTON_PANEL_H
#define BUTTON_PANEL_H

#include <ros/ros.h>
#include <rviz/panel.h>
#include <QVBoxLayout>
#include <QPushButton>
#include <QWidget>

namespace control_gui
{

class ButtonPanel : public rviz::Panel
{
public:
    ButtonPanel(QWidget *parent = 0);

public Q_SLOTS:
    void onNextButtonClicked();
    void onRepeatButtonClicked();
    void onResetButtonClicked();

private:
    ros::NodeHandle node_handle;
    ros::Publisher reset_publisher, next_publisher, repeat_publisher;
};
} // namespace control_gui
#endif // BUTTON_PANEL_H
