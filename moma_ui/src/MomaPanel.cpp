#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>

#include "moma_ui/MomaPanel.h"

namespace moma_ui
{
// - Act as a container for GUI elements QLineEdit.
// - Saving and restoring internal state from a config file.

// We start with the constructor, doing the standard Qt thing of
// passing the optional *parent* argument on to the superclass
// constructor, and also zero-ing the velocities we will be
// publishing.
MomaPanel::MomaPanel(QWidget *parent)
    : rviz::Panel(parent)
{
  // Next we lay out the "output topic" text entry field using a
  // QLabel and a QLineEdit in a QHBoxLayout.
  QHBoxLayout* topic_layout = new QHBoxLayout;
  topic_layout->addWidget( new QLabel( "Output Topic:" ));
  output_topic_editor_ = new QLineEdit;
  topic_layout->addWidget( output_topic_editor_ );

  // Next we make signal/slot connections.
//   connect( drive_widget_, SIGNAL( outputVelocity( float, float )), this, SLOT( setVel( float, float )));
  connect( output_topic_editor_, SIGNAL( editingFinished() ), this, SLOT( updateTopic() ));
//   connect( output_timer, SIGNAL( timeout() ), this, SLOT( sendVel() ));

  // Start the timer.
//   output_timer->start( 100 );

  // Make the control widget start disabled, since we don't start with an output topic.
//   drive_widget_->setEnabled( false );
}

// Set the topic name we are publishing to.
void MomaPanel::setTopic( const QString& new_topic )
{
  // Only take action if the name has changed.
  if( new_topic != output_topic_ )
  {
    output_topic_ = new_topic;
    // If the topic is the empty string, don't publish anything.
    if( output_topic_ == "" )
    {
        ROS_WARN("Output topic is empty");
    //   velocity_publisher_.shutdown();
    }
    else
    {
        ROS_WARN("Output topic is not empty");
      // The old ``velocity_publisher_`` is destroyed by this assignment,
      // and thus the old topic advertisement is removed.  The call to
      // nh_advertise() says we want to publish data on the new topic
      // name.
    //   velocity_publisher_ = nh_.advertise<geometry_msgs::Twist>( output_topic_.toStdString(), 1 );
    }
    // rviz::Panel defines the configChanged() signal.  Emitting it
    // tells RViz that something in this panel has changed that will
    // affect a saved config file.  Ultimately this signal can cause
    // QWidget::setWindowModified(true) to be called on the top-level
    // rviz::VisualizationFrame, which causes a little asterisk ("*")
    // to show in the window's title bar indicating unsaved changes.
    Q_EMIT configChanged();
  }

  // Gray out the control widget when the output topic is empty.
//   drive_widget_->setEnabled( output_topic_ != "" );
}

// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void MomaPanel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
  config.mapSetValue( "Topic", output_topic_ );
}

// Load all configuration data for this panel from the given Config object.
void MomaPanel::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
  QString topic;
  if( config.mapGetString( "Topic", &topic ))
  {
    output_topic_editor_->setText( topic );
    updateTopic();
  }
}
} // end namespace moma_ui

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(moma_ui::MomaPanel, rviz::Panel)
