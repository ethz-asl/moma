#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>

#include <vector>
#include <string>

#include <ros/names.h>

#include "moma_ui/MomaPanel.h"

namespace moma_ui
{
MomaPanel::MomaPanel(QWidget *parent)
    : rviz::Panel(parent)
{

  // rosbag recording
  QHBoxLayout* topic_layout = new QHBoxLayout;
  topic_layout->addWidget( new QLabel( "Bag DIR:" ));
  topic_layout->addWidget( rosbag_output_dir_editor_ );
  topic_layout->addWidget( new QLabel( "Topics:" ));
  topic_layout->addWidget( rosbag_topic_name_editor_ );
  topic_layout->addWidget( rosbag_start_button_ );
  topic_layout->addWidget( rosbag_stop_button_ );
  
  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout( topic_layout );
  setLayout( layout );

  // Next we make signal/slot connections.
  connect( rosbag_topic_name_editor_, SIGNAL( editingFinished() ), this, SLOT( updateBagTopics() ));
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
        ROS_WARN("moma_ui: There are no topics to be recorded!");
    //   velocity_publisher_.shutdown();
    }
    else
    {
        bag_topics_vector_.clear();
        ROS_INFO("moma_ui: There are topics to be recorded!");
        // Split the string using ';' as a delimiter
        QStringList substrings = new_topic.split(";");
        // You can then iterate over the list of substrings or access individual parts
        for(const QString& topic : substrings) {
            ROS_INFO("moma_ui: Topic: %s", topic.toStdString().c_str());
            std::string error;
            if (!ros::names::validate(topic.toStdString(), error)) {
                ROS_ERROR("moma_ui: Invalid topic name: %s", error.c_str());
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
  config.mapSetValue( "Topic", bag_topics_ );
}

// Load all configuration data for this panel from the given Config object.
void MomaPanel::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
  QString topic;
  if( config.mapGetString( "Topic", &topic ))
  {
    rosbag_topic_name_editor_->setText( topic );
    updateBagTopics();
  }
}

void MomaPanel::updateBagTopics()
{
  setBagTopics( rosbag_topic_name_editor_->text() );
}


} // end namespace moma_ui

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(moma_ui::MomaPanel, rviz::Panel)
