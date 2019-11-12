#include "button_panel.h"

namespace control_gui {

ButtonPanel::ButtonPanel(QWidget *parent): rviz::Panel(parent) {
    QVBoxLayout *layout = new QVBoxLayout;
    QPushButton *scan_button = new QPushButton("Scan Scene");
    layout->addWidget(scan_button);
    setLayout(layout);

    connect(scan_button, &QPushButton::clicked, this, &ButtonPanel::onButtonClick);
}

void ButtonPanel::onButtonClick() {
    ROS_INFO("Scan button was clicked.");
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(control_gui::ButtonPanel, rviz::Panel)

