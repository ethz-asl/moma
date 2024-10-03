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
  // In this example setTopic() does not get connected to any signal
  // (it is called directly), but it is easy to define it as a public
  // slot instead of a private function in case it would be useful to
  // some other user.
  void setTopic( const QString& topic );

protected Q_SLOTS:
  // updateTopic() reads the topic name from the QLineEdit and calls
  // setTopic() with the result.
  void updateTopic();

protected:
    // One-line text editor for entering the outgoing ROS topic name.
    QLineEdit* output_topic_editor_;

    // The current name of the output topic.
    QString output_topic_;

    QPushButton* btn_next_;


    // ROS node handle (for any communication, if needed)
    ros::NodeHandle nh_;

};

} // end namespace moma_ui

#endif // BUTTON_PANEL_H
