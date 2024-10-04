#ifndef MOMA_PANEL_H
#define MOMA_PANEL_H

#include <ros/ros.h>
#include <rviz/panel.h>
#include <QLineEdit>
#include <QPushButton>

namespace moma_ui
{
class MomaPanel : public rviz::Panel
{
    Q_OBJECT
public:
    MomaPanel(QWidget *parent = 0);

    // Save and load settings
    virtual void load(const rviz::Config &config);
    virtual void save(rviz::Config config) const;

public Q_SLOTS:
  void setTopic( const QString& topic );

protected Q_SLOTS:
  void updateTopic();

protected:
    // One-line text editor for entering the outgoing ROS topic name.
    QLineEdit* output_topic_editor_;
    // The current name of the output topic.
    QString output_topic_;
    // ROS node handle (for any communication, if needed)
    ros::NodeHandle nh_;

};

} // end namespace moma_ui

#endif // BUTTON_PANEL_H
