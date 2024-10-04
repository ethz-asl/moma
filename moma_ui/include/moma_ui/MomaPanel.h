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
  void setBagDir( const QString& dir );
  void setBagTopics( const QString& topic );

protected Q_SLOTS:
  void updateBagDir();
  void updateBagTopics();

protected:    
    // ROS node handle (for any communication, if needed)
    ros::NodeHandle nh_;

    // ROSBAG RECORD
    // Qt elements
    QLineEdit* rosbag_output_dir_editor_ = new QLineEdit;
    QLineEdit* rosbag_topic_name_editor_ = new QLineEdit;
    QPushButton* rosbag_start_button_ = new QPushButton("Start rosbag");
    QPushButton* rosbag_stop_button_ = new QPushButton("Stop rosbag");
    // internal variables
    QString bag_output_dir_;
    QString bag_topics_;
    std::vector<std::string> bag_topics_vector_;
};

} // end namespace moma_ui

#endif // BUTTON_PANEL_H
