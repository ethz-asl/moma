#ifndef MOMA_PANEL_H
#define MOMA_PANEL_H

#include <ros/ros.h>
#include <rviz/panel.h>
#include <QLineEdit>
#include <QPushButton>
#include <QCheckBox>

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
  void resetSam();
  void updateSamFgMinH();
  void runSam();
  void updateBagDir();
  void updateBagTopics();
  void toggledFgSam();


protected:    
    // ROS node handle (for any communication, if needed)
    ros::NodeHandle nh_;
    ros::Publisher fg_min_height_pub_;

    // PERCEPTION
    // Qt elements
    QPushButton* perception_bbox_button_ = new QPushButton("Set BBOX");
    QPushButton* perception_detect_plane_button_ = new QPushButton("Detect workplane");
    QPushButton* perception_clear_map_button_ = new QPushButton("Clear map");
    
    // SAM
    // Qt elements
    QCheckBox* sam_fg_toggle = new QCheckBox("FG/BG");
    QLineEdit* sam_label_editor_ = new QLineEdit;
    QPushButton* sam_reset_label_ctrlpts_button_ = new QPushButton("Reset");
    QPushButton* sam_run_button_ = new QPushButton("Segment");

    // ROSBAG
    // Qt elements
    QLineEdit* rosbag_output_dir_editor_ = new QLineEdit;
    QLineEdit* rosbag_topic_name_editor_ = new QLineEdit;
    QPushButton* rosbag_start_button_ = new QPushButton("Start rosbag");
    QPushButton* rosbag_stop_button_ = new QPushButton("Stop rosbag");
    // internal variables
    QString bag_output_dir_;
    QString bag_topics_;
    std::vector<std::string> bag_topics_vector_;

    // P2P
    // Qt elements
    QLineEdit* p2p_label_ = new QLineEdit;
    QPushButton* p2p_store_ee_pose_button_ = new QPushButton("Store EE Pose");
    QPushButton* p2p_store_joints_button_ = new QPushButton("Store Joints");
    QPushButton* p2p_goto_label_button_ = new QPushButton("Go to Label");
    QPushButton* p2p_delete_label_button_ = new QPushButton("Delete Label");

    // TRJ
    // Qt elements
    QPushButton* trj_select_start_button_ = new QPushButton("Select TRJ start");
    QPushButton* trj_select_end_button_ = new QPushButton("Select TRJ end");
    QLineEdit* trj_set_height_editor_ = new QLineEdit;
    QPushButton* trj_execute_button_ = new QPushButton("Execute TRJ");

};

} // end namespace moma_ui

#endif // BUTTON_PANEL_H
