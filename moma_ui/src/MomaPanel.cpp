#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>

#include <vector>
#include <string>

#include <ros/names.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>

#include "moma_ui/MomaPanel.h"

namespace moma_ui
{
MomaPanel::MomaPanel(QWidget *parent)
    : rviz::Panel(parent)
{
    // PERCEPTION
    // Set up the layout for the perception buttons
    QHBoxLayout* perception_layout = new QHBoxLayout;
    perception_layout->addWidget( new QLabel( "<b>PERCEPTION</b>" ));
    perception_layout->addWidget( perception_bbox_button_ );
    perception_layout->addWidget( perception_detect_plane_button_ );
    perception_layout->addWidget( perception_clear_map_button_ );

    // SAM
    // Set up the layout for the SAM buttons
    QHBoxLayout* sam_layout = new QHBoxLayout;
    sam_layout->addWidget( new QLabel( "<b>SEGMENTATION</b>" ));
    sam_layout->addWidget( sam_fg_toggle );
    sam_layout->addWidget( new QLabel( "FG Min. Height:" ));
    sam_layout->addWidget( sam_min_height_editor_ );
    sam_layout->addWidget( sam_reset_label_ctrlpts_button_ );
    sam_layout->addWidget( sam_run_button_ );

  // ROSBAG
  QHBoxLayout* topic_layout = new QHBoxLayout;
      topic_layout->addWidget( new QLabel( "<b>LOGGING</b>" ));

  topic_layout->addWidget( new QLabel( "Bag DIR:" ));
  topic_layout->addWidget( rosbag_output_dir_editor_ );
  topic_layout->addWidget( new QLabel( "Topics:" ));
  topic_layout->addWidget( rosbag_topic_name_editor_ );
  topic_layout->addWidget( rosbag_start_button_ );
  topic_layout->addWidget( rosbag_stop_button_ );
  
  // P2P
    QHBoxLayout* p2p_layout = new QHBoxLayout;
    p2p_layout->addWidget( new QLabel( "<b>POINT-TO-POINT</b>" ));
    p2p_layout->addWidget( new QLabel( "Label:" ));
    p2p_layout->addWidget( p2p_label_ );
    p2p_layout->addWidget( p2p_store_ee_pose_button_ );
    p2p_layout->addWidget( p2p_store_joints_button_ );
    p2p_layout->addWidget( p2p_goto_label_button_ );
    p2p_layout->addWidget( p2p_delete_label_button_ );

    // TRJ
    // Set up the layout for the trajectory buttons
    QHBoxLayout* trj_layout = new QHBoxLayout;
              trj_layout->addWidget( new QLabel( "<b>TRAJECTORY</b>" ));

    trj_layout->addWidget( trj_select_start_button_ );
    trj_layout->addWidget( trj_select_end_button_ );
    trj_layout->addWidget( new QLabel( "Trajectory Height:" ));
    trj_layout->addWidget( trj_execute_button_ );


  QVBoxLayout* layout = new QVBoxLayout;
    layout->addLayout( perception_layout );
    layout->addLayout( sam_layout );
  layout->addLayout( topic_layout );
    layout->addLayout( p2p_layout );
    layout->addLayout( trj_layout );
  setLayout( layout );

  // Next we make signal/slot connections.
  connect( sam_reset_label_ctrlpts_button_, SIGNAL( clicked() ), this, SLOT( resetSam() ));
  connect( sam_fg_toggle, SIGNAL( stateChanged(int) ), this, SLOT( toggledFgSam() ));
  connect( sam_run_button_, SIGNAL( clicked() ), this, SLOT( runSam() ));
  connect( sam_min_height_editor_, SIGNAL( editingFinished() ), this, SLOT( updateSamFgMinH() ));
  connect( rosbag_output_dir_editor_, SIGNAL( editingFinished() ), this, SLOT( updateBagDir() ));
  connect( rosbag_topic_name_editor_, SIGNAL( editingFinished() ), this, SLOT( updateBagTopics() ));

  // other stuff
  fg_min_height_pub_ = nh_.advertise<std_msgs::Float32>("moma_ui/sam/foreground_min_height", 1);
}

// Set the directory where the bag file will be saved.
void MomaPanel::setBagDir( const QString& new_dir )
{
  // Only take action if the name has changed.
  if( new_dir != bag_output_dir_ )
  {
    ROS_INFO("moma_panel: Setting bag output directory to %s", new_dir.toStdString().c_str());
    bag_output_dir_ = new_dir;
    Q_EMIT configChanged();
  }
}

// Set the topic name we are publishing to.
void MomaPanel::setBagTopics( const QString& new_topic )
{
  // Only take action if the name has changed.
  if( new_topic != bag_topics_ )
  {
    bag_topics_ = new_topic;
    // If the topic is the empty string, don't publish anything.
    if( bag_topics_ == "" )
    {
        ROS_WARN("moma_panel: There are no topics to be recorded!");
    //   velocity_publisher_.shutdown();
    }
    else
    {
        bag_topics_vector_.clear();
        ROS_INFO("moma_panel: There are topics to be recorded!");
        // Split the string using ';' as a delimiter
        QStringList substrings = new_topic.split(";");
        // You can then iterate over the list of substrings or access individual parts
        for(const QString& topic : substrings) {
            ROS_INFO("moma_panel: Topic: %s", topic.toStdString().c_str());
            std::string error;
            if (!ros::names::validate(topic.toStdString(), error)) {
                ROS_ERROR("moma_panel: Invalid topic name: %s", error.c_str());
                continue;
            }
            bag_topics_vector_.push_back(topic.toStdString());
        }
        nh_.setParam("moma_ui/topics_to_record", bag_topics_vector_);  // Store the list under the param name "topics_to_record"


    }
    Q_EMIT configChanged();
  }
}

// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void MomaPanel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
  config.mapSetValue( "BagOutDir", bag_output_dir_ );
  config.mapSetValue( "BagTopics", bag_topics_ );
  config.mapSetValue( "FGMinHeight", sam_min_height_editor_->text() );
}

// Load all configuration data for this panel from the given Config object.
void MomaPanel::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
  QString dir;
  if( config.mapGetString( "BagOutDir", &dir ))
  {
    rosbag_output_dir_editor_->setText( dir );
    updateBagDir();
}
  QString topic;
  if( config.mapGetString( "BagTopics", &topic ))
  {
    rosbag_topic_name_editor_->setText( topic );
    updateBagTopics();
  }

  QString fg_min_height;
  if( config.mapGetString( "FGMinHeight", &fg_min_height ))
  {
    sam_min_height_editor_->setText( fg_min_height );
    updateSamFgMinH();
  }
}

void MomaPanel::resetSam()
{
    // Reset the control points of the SAM
    // Call the service to reset the control points
    ros::ServiceClient client = nh_.serviceClient<std_srvs::Empty>("moma_ui/sam/reset");
    std_srvs::Empty srv;
    if (client.call(srv))
    {
        ROS_INFO("moma_panel: SAM control points have been reset");
    }
    else
    {
        ROS_ERROR("moma_panel: Failed to reset SAM control points");
    }
}

void MomaPanel::runSam()
{
    // Run SAM
    ROS_INFO("moma_panel: Sending request to SAM...");
    // Call the service to run the SAM
    ros::ServiceClient client = nh_.serviceClient<std_srvs::Trigger>("moma_ui/sam/run");
    std_srvs::Trigger srv;
    client.call(srv);
    if (srv.response.success)
    {
        ROS_INFO("moma_panel: SAM has been run: %s", srv.response.message.c_str());
    }
    else
    {
        ROS_WARN("moma_panel: Failed to run SAM: %s", srv.response.message.c_str());
    }
}

void MomaPanel::toggledFgSam()
{
    bool fg = sam_fg_toggle->isChecked();
    ros::ServiceClient client = nh_.serviceClient<std_srvs::SetBool>("moma_ui/sam/set_label_fg_bg");
    std_srvs::SetBool srv;
    srv.request.data = fg;
    client.call(srv);
    if (!srv.response.success)
    {
        ROS_WARN("moma_panel: Failed to toggle SAM FG/BG: %s", srv.response.message.c_str());
    }
}

void MomaPanel::updateSamFgMinH()
{
    float fg_min_height = sam_min_height_editor_->text().toFloat();
    std_msgs::Float32 msg;
    msg.data = fg_min_height;
    fg_min_height_pub_.publish(msg);
}

void MomaPanel::updateBagDir()
{
    setBagDir( rosbag_output_dir_editor_->text() );
} 

void MomaPanel::updateBagTopics()
{
  setBagTopics( rosbag_topic_name_editor_->text() );
}

}
// end namespace moma_ui

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(moma_ui::MomaPanel, rviz::Panel)
