#pragma once

#ifndef Q_MOC_RUN
// qt
#include <QPainter>
#include <QCheckBox>
#include <QPushButton>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTableWidget>
#include <QComboBox>
#include <QTimer>

// ros
#include <ros/ros.h>
#include <rviz/panel.h>
#include <pluginlib/class_list_macros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

// messages
#include <visualization_msgs/Marker.h>

// gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

// std
#include <thread>

#endif

using namespace gazebo;
using namespace std;

class ShoulderJointPoseEstimator: public rviz::Panel{
    Q_OBJECT
public:
    ShoulderJointPoseEstimator(QWidget *parent = 0);
    ~ShoulderJointPoseEstimator();

    /**
     * Load all configuration data for this panel from the given Config object.
     * @param config rviz config file
     */
    virtual void load(const rviz::Config &config);

    /**
     * Save all configuration data from this panel to the given
     * Config object.  It is important here that you call save()
     * on the parent class so the class id and panel name get saved.
     * @param config rviz config file
     */
    virtual void save(rviz::Config config) const;

private:

    void updateMouse();

    bool update = true;

    ros::NodeHandlePtr nh;
    boost::shared_ptr<ros::AsyncSpinner> spinner;
    std::thread *updateThread;
    ros::Publisher marker_visualization_pub;
};