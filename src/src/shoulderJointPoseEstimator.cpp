#include "shoulderJointPoseEstimator.hpp"

ShoulderJointPoseEstimator::ShoulderJointPoseEstimator(QWidget *parent) : rviz::Panel(parent) {
    // Create the main layout
    QHBoxLayout *mainLayout = new QHBoxLayout;

    // Create the frame to hold all the widgets
    QFrame *mainFrame = new QFrame();

    QVBoxLayout *frameLayout = new QVBoxLayout();

    QHBoxLayout *roboyIdlayout = new QHBoxLayout();
    QLabel *roboyIDlabel = new QLabel(tr("roboy ID:"));
    frameLayout->addWidget(roboyIDlabel);

//        QComboBox *roboyID = new QComboBox();
//        roboyID->setObjectName("roboyID");
//        connect(roboyID, SIGNAL(currentIndexChanged(int)), this, SLOT(changeID(int)));
//        roboyIdlayout->addWidget(roboyID);

    // Add frameLayout to the frame
    mainFrame->setLayout(frameLayout);

    // Add the frame to the main layout
    mainLayout->addWidget(mainFrame);

    // Remove margins to reduce space
    frameLayout->setContentsMargins(0, 0, 0, 0);
    mainLayout->setContentsMargins(0, 0, 0, 0);

    this->setLayout(mainLayout);

    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "RvizShoulderPoseEstimator",
                  ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
    }

    nh = ros::NodeHandlePtr(new ros::NodeHandle);

    spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1));

    marker_visualization_pub = nh->advertise<visualization_msgs::Marker>("visualization_marker", 100);

    updateThread = new std::thread( &ShoulderJointPoseEstimator::updateMouse, this);
}

ShoulderJointPoseEstimator::~ShoulderJointPoseEstimator(){
    update = false;
    updateThread->join();
    delete updateThread;
}

void ShoulderJointPoseEstimator::load(const rviz::Config &config){
//    rviz::Panel::load(config);
//    QCheckBox* w = this->findChild<QCheckBox*>("visualizeMesh");
//    bool checked = false;
//    config.mapGetBool(w->objectName(), &checked);
//    w->setChecked(checked);
}

void ShoulderJointPoseEstimator::save(rviz::Config config) const{
//    QCheckBox* w = this->findChild<QCheckBox*>("visualizeMesh");
//    config.mapSetValue(w->objectName(), w->isChecked());
//    rviz::Panel::save(config);
}

void ShoulderJointPoseEstimator::updateMouse(){
    double sphere_radius = 0.03; // in meter
    math::Vector3 positionOnSphere(0,0,0.03);
    math::Quaternion pose;
    while(update){
        // get mouse updates here and put the into dv
        math::Vector3 dv(0.001, 0.002, 0);
        math::Vector3 rotationAxis = positionOnSphere.Cross(dv);
        rotationAxis-=positionOnSphere;
        math::Quaternion q(rotationAxis, tanh(dv.GetLength()/sphere_radius));
        pose *= q;

        // publish new pose
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.ns = "sphere joint";
        marker.type = visualization_msgs::Marker::CUBE;
        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration(0);
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.action = visualization_msgs::Marker::ADD;
        marker.header.stamp = ros::Time::now();
        marker.points.clear();
        marker.id = 0;
        marker.pose.orientation.w = pose.w;
        marker.pose.orientation.x = pose.x;
        marker.pose.orientation.y = pose.y;
        marker.pose.orientation.z = pose.z;
        marker_visualization_pub.publish(marker);
    }
}

PLUGINLIB_EXPORT_CLASS(ShoulderJointPoseEstimator, rviz::Panel)