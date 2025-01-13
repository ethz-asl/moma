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

#include <std_msgs/Bool.h>

#include "moma_ui/MomaPanel.h"


// franka_msgs/ErrorRecoveryActionGoal
#include <franka_msgs/ErrorRecoveryActionGoal.h>

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
    // enable by default
    sam_fg_toggle->setChecked(true);
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

    // SWEEP
    // Set up the layout for the trajectory buttons
    QHBoxLayout* sweep_layout = new QHBoxLayout;
              sweep_layout->addWidget( new QLabel( "<b>SWEEP</b>" ));

    sweep_layout->addWidget( new QLabel( "Sweep from topic" ));
    sweep_layout->addWidget( sweep_topic_toggle );
    sweep_layout->addWidget( sweep_select_start_button_ );
    sweep_layout->addWidget( sweep_select_end_button_ );
    sweep_layout->addWidget( new QLabel( "Sweep Height:" ));
    sweep_layout->addWidget( sweep_execute_button_ );
    sweep_topic_toggle->setChecked(true);

    // TEACH&REPEAT
    // Set up the layout for the trajectory buttons
    QHBoxLayout* teach_repeat_layout = new QHBoxLayout;
    teach_repeat_layout->addWidget( new QLabel( "<b>TEACH&REPEAT</b>" ));
    teach_repeat_layout->addWidget( new QLabel( "Teach Trajectory" ));
    teach_repeat_layout->addWidget( teach_repeat_label_ );
    teach_repeat_layout->addWidget( teach_repeat_start_button_ );
    teach_repeat_layout->addWidget( teach_repeat_stop_button_ );
    teach_repeat_layout->addWidget( teach_repeat_execute_button_ );


    // MOVE IT COMMANDER
    QHBoxLayout* move_it_cmd_layout = new QHBoxLayout;
    move_it_cmd_layout->addWidget( new QLabel( "<b>MOVEIT CMD BASIC</b>" ));
    move_it_cmd_layout->addWidget( moveit_cmd_toggle_cmd_input_ );
    moveit_cmd_toggle_cmd_input_->setChecked(true);
    move_it_cmd_layout->addWidget( moveit_cmd_execute_path_button_ );
    move_it_cmd_layout->addWidget( moveit_reset_button_ );
    move_it_cmd_layout->addWidget( new QLabel( "Offset tx:" ));
    move_it_cmd_layout->addWidget( moveit_offset_tx_ );
    move_it_cmd_layout->addWidget( new QLabel( "Offset ty:" ));
    move_it_cmd_layout->addWidget( moveit_offset_ty_ );
    move_it_cmd_layout->addWidget( new QLabel( "Offset tz:" ));
    move_it_cmd_layout->addWidget( moveit_offset_tz_ );
    
    // GOTO
    // Set up the layout for the trajectory buttons
    QHBoxLayout* goto_layout = new QHBoxLayout;
    goto_layout->addWidget( new QLabel( "<b>MOVEIT CMD GOTO</b>" ));
    goto_layout->addWidget( new QLabel( "Label:" ));
    goto_layout->addWidget( goto_label_ );
    goto_layout->addWidget( go_to_button_ );
    goto_layout->addWidget( go_to_store_ee_pose_button_ );
    goto_layout->addWidget( go_to_store_joints_button_ );
    goto_layout->addWidget( go_to_delete_label_button_ );
    goto_layout->addWidget( go_to_clear_all_labels_button_ );


    // TASK
    // Set up the layout for the task buttons
    QHBoxLayout* task_layout = new QHBoxLayout;
    task_layout->addWidget( new QLabel( "<b>TASK</b>" ));
    task_layout->addWidget( task_plan_button_ );
    task_layout->addWidget( task_execute_button_ );


  // Now we set the main layout for the panel.
  QVBoxLayout* layout = new QVBoxLayout;
    layout->addLayout( perception_layout );
    layout->addLayout( sam_layout );
  layout->addLayout( topic_layout );
    // layout->addLayout( p2p_layout );
    // layout->addLayout( sweep_layout );
    // layout->addLayout( teach_repeat_layout );
    layout->addLayout( move_it_cmd_layout );
    layout->addLayout( goto_layout );
    // layout->addLayout( task_layout );
  setLayout( layout );

  // Next we make signal/slot connections.
  connect( perception_detect_plane_button_, SIGNAL( clicked() ), this, SLOT( detectPlane() ));
  connect( sam_reset_label_ctrlpts_button_, SIGNAL( clicked() ), this, SLOT( resetSam() ));
  connect( sam_fg_toggle, SIGNAL( stateChanged(int) ), this, SLOT( toggledFgSam() ));
  connect( sam_run_button_, SIGNAL( clicked() ), this, SLOT( runSam() ));
  connect( sam_min_height_editor_, SIGNAL( editingFinished() ), this, SLOT( updateSamFgMinH() ));
  connect( rosbag_output_dir_editor_, SIGNAL( editingFinished() ), this, SLOT( updateBagDir() ));
  connect( rosbag_topic_name_editor_, SIGNAL( editingFinished() ), this, SLOT( updateBagTopics() ));
  connect( perception_clear_map_button_, SIGNAL( clicked() ), this, SLOT( clearMap() ));
  connect( rosbag_start_button_, SIGNAL( clicked() ), this, SLOT( startRosbag() ));
  connect( rosbag_stop_button_, SIGNAL( clicked() ), this, SLOT( stopRosbag() ));  
  connect( task_plan_button_, SIGNAL( clicked() ), this, SLOT( planTask() ));
  connect( sweep_topic_toggle, SIGNAL( stateChanged(int) ), this, SLOT( toggleSweepTopic() ));

    connect( moveit_cmd_toggle_cmd_input_, SIGNAL( stateChanged(int) ), this, SLOT( toggleMoveitCmdInput() ));
    connect( moveit_cmd_execute_path_button_, SIGNAL( clicked() ), this, SLOT( executeMoveitPath() ));
  connect( moveit_reset_button_, SIGNAL( clicked() ), this, SLOT( resetMoveit() ));
  connect( moveit_offset_tx_, SIGNAL( editingFinished() ), this, SLOT( updateMoveitOffsetX() ));
    connect( moveit_offset_ty_, SIGNAL( editingFinished() ), this, SLOT( updateMoveitOffsetY() ));
    connect( moveit_offset_tz_, SIGNAL( editingFinished() ), this, SLOT( updateMoveitOffsetZ() ));

  connect( goto_label_ , SIGNAL( editingFinished() ), this, SLOT( goToUpdateLabel() ));
  connect( go_to_button_, SIGNAL( clicked() ), this, SLOT( gotoLabel() ));
  connect( go_to_store_ee_pose_button_, SIGNAL( clicked() ), this, SLOT( goToStoreEePose() ));
  connect( go_to_store_joints_button_, SIGNAL( clicked() ), this, SLOT( goToStoreJoints() ));
  connect( go_to_delete_label_button_, SIGNAL( clicked() ), this, SLOT( goToDeleteLabel() ));
  connect( go_to_clear_all_labels_button_, SIGNAL( clicked() ), this, SLOT( goToClearAllLabels() ));

  connect( task_execute_button_, SIGNAL( clicked() ), this, SLOT( taskExecute() ));


  // other stuff
  fg_min_height_pub_ = nh_.advertise<std_msgs::Float32>("moma_ui/sam/foreground_min_height", 1);
  go_to_label_pub_ = nh_.advertise<std_msgs::String>("moma_ui/commander/label", 1);
    error_recovery_moveit_pub_ = nh_.advertise<franka_msgs::ErrorRecoveryActionGoal>("/franka_control/error_recovery/goal", 1);
    moveit_offset_x_pub_ = nh_.advertise<std_msgs::Float32>("moma_ui/commander/ee_offset_t_x", 1);
    moveit_offset_y_pub_ = nh_.advertise<std_msgs::Float32>("moma_ui/commander/ee_offset_t_y", 1);
    moveit_offset_z_pub_ = nh_.advertise<std_msgs::Float32>("moma_ui/commander/ee_offset_t_z", 1);

    moveit_trigger_pub_ = nh_.advertise<std_msgs::Bool>("moma_ui/commander/execute_path_topic", 1);

}

void MomaPanel::updateMoveitOffsetX()
{
    float offset_x = moveit_offset_tx_->text().toFloat();
    std_msgs::Float32 msg;
    msg.data = offset_x;
    moveit_offset_x_pub_.publish(msg);
}

void MomaPanel::updateMoveitOffsetY()
{
    float offset_y = moveit_offset_ty_->text().toFloat();
    std_msgs::Float32 msg;
    msg.data = offset_y;
    moveit_offset_y_pub_.publish(msg);
}

void MomaPanel::updateMoveitOffsetZ()
{
    float offset_z = moveit_offset_tz_->text().toFloat();
    std_msgs::Float32 msg;
    msg.data = offset_z;
    moveit_offset_z_pub_.publish(msg);
}


void MomaPanel::toggleMoveitCmdInput()
{
    bool cmd_input = moveit_cmd_toggle_cmd_input_->isChecked();
    ros::ServiceClient client = nh_.serviceClient<std_srvs::SetBool>("moma_ui/commander/toggle_cmd_input");
    std_srvs::SetBool srv;
    srv.request.data = cmd_input;
    client.call(srv);
    if (!srv.response.success)
    {
        ROS_WARN("moma_panel: Failed to toggle MoveIt command input: %s", srv.response.message.c_str());
    }
}

void MomaPanel::executeMoveitPath()
{
    ROS_WARN("moma_panel: Executing MoveIt path");
    // ros::ServiceClient client = nh_.serviceClient<std_srvs::Trigger>("moma_ui/commander/execute_path");
    // std_srvs::Trigger srv;
    // if (client.call(srv))
    // {
    //     ROS_INFO("moma_panel: MoveIt path execution service has been called");
    // }
    // else
    // {
    //     ROS_ERROR("moma_panel: Failed to call MoveIt path execution service");
    // }

    // Publish true to moma_ui/commander/execute_path
    std_msgs::Bool msg;
    msg.data = true;
    // create a publisher
    moveit_trigger_pub_.publish(msg);
}



void MomaPanel::resetMoveit()
{
    ROS_WARN("moma_panel: Resetting MoveIt");
    franka_msgs::ErrorRecoveryActionGoal msg;
    error_recovery_moveit_pub_.publish(msg);
    // Also make a service call to moma_ui/commander/delete_waypoints of type std_srvs/Trigger
    ros::ServiceClient client = nh_.serviceClient<std_srvs::Trigger>("moma_ui/commander/delete_waypoints");
    std_srvs::Trigger srv;
    if (client.call(srv))
    {
        ROS_INFO("moma_panel: Waypoints have been deleted");
    }
    else
    {
        ROS_ERROR("moma_panel: Failed to delete waypoints");
    }

}


void MomaPanel::goToUpdateLabel()
{
    ROS_WARN("moma_panel: Updating go to label");
    std_msgs::String msg;
    msg.data = goto_label_->text().toStdString();
    go_to_label_pub_.publish(msg);
}

void MomaPanel::gotoLabel()
{
    ROS_WARN("moma_panel: Sending go to label");
    ros::ServiceClient client = nh_.serviceClient<std_srvs::Trigger>("moma_ui/commander/goto_label");
    std_srvs::Trigger srv;
    if (client.call(srv))
    {
        ROS_INFO("moma_panel: Go to label service has been called");
    }
    else
    {
        ROS_ERROR("moma_panel: Failed to call go to label service");
    }
}

void MomaPanel::goToStoreEePose()
{
    ROS_WARN("moma_panel: Storing EE pose");
    ros::ServiceClient client = nh_.serviceClient<std_srvs::Trigger>("moma_ui/commander/store_pose");
    std_srvs::Trigger srv;
    if (client.call(srv))
    {
        ROS_INFO("moma_panel: Store EE pose service has been called");
    }
    else
    {
        ROS_ERROR("moma_panel: Failed to call store EE pose service");
    }
}

void MomaPanel::goToStoreJoints()
{
    ROS_WARN("moma_panel: Storing joints");
    ros::ServiceClient client = nh_.serviceClient<std_srvs::Trigger>("moma_ui/commander/store_joint_state");
    std_srvs::Trigger srv;
    if (client.call(srv))
    {
        ROS_INFO("moma_panel: Store joints service has been called");
    }
    else
    {
        ROS_ERROR("moma_panel: Failed to call store joints service");
    }
}

void MomaPanel::goToDeleteLabel()
{
    ROS_WARN("moma_panel: Deleting label");
    ros::ServiceClient client = nh_.serviceClient<std_srvs::Trigger>("moma_ui/commander/delete_label");
    std_srvs::Trigger srv;
    if (client.call(srv))
    {
        ROS_INFO("moma_panel: Delete label service has been called");
    }
    else
    {
        ROS_ERROR("moma_panel: Failed to call delete label service");
    }
}

void MomaPanel::goToClearAllLabels()
{
    ROS_WARN("moma_panel: Clearing all labels");
    ros::ServiceClient client = nh_.serviceClient<std_srvs::Trigger>("moma_ui/commander/delete_all_labels");
    std_srvs::Trigger srv;
    if (client.call(srv))
    {
        ROS_INFO("moma_panel: Clear all labels service has been called");
    }
    else
    {
        ROS_ERROR("moma_panel: Failed to call clear all labels service");
    }
}

void MomaPanel::toggleSweepTopic()
{
    bool sweep = sweep_topic_toggle->isChecked();
    ros::ServiceClient client = nh_.serviceClient<std_srvs::SetBool>("moma_ui/sweep/use_sweep_topic");
    std_srvs::SetBool srv;
    srv.request.data = sweep;
    client.call(srv);
    if (!srv.response.success)
    {
        ROS_WARN("moma_panel: Failed to toggle sweep topic: %s", srv.response.message.c_str());
    }
}



void MomaPanel::detectPlane()
{
    // Call the service to detect the plane
    ros::ServiceClient client = nh_.serviceClient<std_srvs::Trigger>("moma_ui/work_plane/detect");
    std_srvs::Trigger srv;
    if (client.call(srv))
    {
        ROS_INFO("moma_panel: Plane detection service has been called");
    }
    else
    {
        ROS_ERROR("moma_panel: Failed to call plane detection service");
    }
}

void MomaPanel::startRosbag()
{
    // Start recording the bag file
    ROS_INFO("moma_panel: Starting rosbag recording...");
    // Call the service to start recording the bag file
    ros::ServiceClient client = nh_.serviceClient<std_srvs::SetBool>("moma_ui/rosbag_recorder/start_stop");
    std_srvs::SetBool srv;
    srv.request.data = true;
    client.call(srv);
    if (srv.response.success)
    {
        ROS_INFO("moma_panel: Rosbag recording has started: %s", srv.response.message.c_str());
    }
    else
    {
        ROS_WARN("moma_panel: Failed to start rosbag recording: %s", srv.response.message.c_str());
    }
}

void MomaPanel::stopRosbag()
{
    // Stop recording the bag file
    ROS_INFO("moma_panel: Stopping rosbag recording...");
    // Call the service to stop recording the bag file
    ros::ServiceClient client = nh_.serviceClient<std_srvs::SetBool>("moma_ui/rosbag_recorder/start_stop");
    std_srvs::SetBool srv;
    srv.request.data = false;
    client.call(srv);
    if (srv.response.success)
    {
        ROS_INFO("moma_panel: Rosbag recording has stopped: %s", srv.response.message.c_str());
    }
    else
    {
        ROS_WARN("moma_panel: Failed to stop rosbag recording: %s", srv.response.message.c_str());
    }
}

void MomaPanel::clearMap()
{
    // Call the service to clear the map
    ros::ServiceClient client = nh_.serviceClient<std_srvs::Trigger>("moma_ui/map/clear");
    std_srvs::Trigger srv;
    if (client.call(srv))
    {
        ROS_INFO("moma_panel: Map has been cleared");
    }
    else
    {
        ROS_ERROR("moma_panel: Failed to clear the map");
    }
}

// Set the directory where the bag file will be saved.
void MomaPanel::setBagDir( const QString& new_dir )
{
  // Only take action if the name has changed.
  if( new_dir != bag_output_dir_ )
  {
    ROS_INFO("moma_panel: Setting bag output directory to %s", new_dir.toStdString().c_str());
    bag_output_dir_ = new_dir;
    nh_.setParam("moma_ui/rosbag_recorder/bag_output_dir", bag_output_dir_.toStdString());  // Store the directory under the param name "bag_output_dir"
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
        nh_.setParam("moma_ui/rosbag_recorder/topics_to_record", bag_topics_vector_);  // Store the list under the param name "topics_to_record"


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
