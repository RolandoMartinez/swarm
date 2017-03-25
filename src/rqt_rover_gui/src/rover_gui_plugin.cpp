// Author: Matthew Fricke
// E-mail: matthew@fricke.co.uk
// Date: 9-16-205
// Purpose: implementation of a simple graphical front end for the UNM-NASA Swarmathon rovers.
// License: GPL3

#include <rover_gui_plugin.h>
<<<<<<< HEAD
#include <Version.h>
=======
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
#include <pluginlib/class_list_macros.h>
#include <QDir>
#include <QtXml>
#include <QFile>
<<<<<<< HEAD
=======
#include <QListWidget>
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
#include <QScrollBar>
#include <QProcess>
#include <QPalette>
#include <QTabBar>
#include <QTabWidget>
#include <QCheckBox>
#include <QRadioButton>
#include <QButtonGroup>
#include <QMessageBox>
#include <QProgressDialog>
#include <QStringList>
#include <QLCDNumber>
<<<<<<< HEAD
#include <QFileDialog>
=======
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
#include <QComboBox>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>
#include <algorithm>

#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp>

//#include <regex> // For regex expressions

<<<<<<< HEAD
#include "MapData.h"

=======
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>

using namespace std;

using boost::property_tree::ptree;

namespace rqt_rover_gui 
{
<<<<<<< HEAD
  RoverGUIPlugin::RoverGUIPlugin() :
      rqt_gui_cpp::Plugin(),
      widget(0),
      disconnect_threshold(5.0), // Rovers are marked as diconnected if they haven't sent a status message for 5 seconds
      current_simulated_time_in_seconds(0.0),
      last_current_time_update_in_seconds(0.0),
      timer_start_time_in_seconds(0.0),
      timer_stop_time_in_seconds(0.0),
      is_timer_on(false)
  {
    setObjectName("RoverGUI");
    info_log_messages = "";
    diag_log_messages = "";

    // Arbitrarily chosen 10000. These values should be set after experimentation.
    max_info_log_length = 10000;
    max_diag_log_length = 10000;

    joy_process = NULL;
    joystickGripperInterface = NULL;
=======
  RoverGUIPlugin::RoverGUIPlugin() : rqt_gui_cpp::Plugin(), widget(0)
  {
    setObjectName("RoverGUI");
    log_messages = "";
    joy_process = NULL;
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea

    obstacle_call_count = 0;

    arena_dim = 20;

    display_sim_visualization = false;

    // Set object clearance values: radius in meters. Values taken from the max dimension of the gazebo collision box for the object.
    // So one half the distance from two opposing corners of the bounding box.
    // In the case of the collection disk a bounding circle is used which gives the radius directly.
    // Values are rounded up to the nearest 10cm.
    target_cluster_size_64_clearance = 0.8;
    target_cluster_size_16_clearance = 0.6;
    target_cluster_size_4_clearance = 0.2;
    target_cluster_size_1_clearance = 0.1;
    rover_clearance = 0.4;
    collection_disk_clearance = 0.5;

    barrier_clearance = 0.5; // Used to prevent targets being placed to close to walls

<<<<<<< HEAD
    map_data = new MapData();
=======
	//Initialize AprilTag detection apparatus
	tf = tag36h11_create();
    td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);

    //Allocate image memory up front so it doesn't need to be done for every image frame
    u8_image = image_u8_create(320, 240);
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
  }

  void RoverGUIPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
  {
    cout << "Rover GUI Starting..." << endl;

    QStringList argv = context.argv();

    widget = new QWidget();

    ui.setupUi(widget);
    
    context.addWidget(widget);

    // Next two lines allow us to catch keyboard input
    widget->installEventFilter(this);
    widget->setFocus();

    // GIT_VERSION is passed in as a compile time definition (see CMakeLists.txt). The version is taken from the last git tag.
<<<<<<< HEAD
    QString version_qstr("<font color='white'>"+GIT_VERSION+"</font>");
    ui.version_number_label->setText(version_qstr);

    widget->setWindowTitle("Rover Interface: Built on " + BUILD_TIME + " Branch: " + GIT_BRANCH);
=======
    QString version_qstr("<font color='white'>"+QString::fromUtf8(GIT_VERSION)+"</font>");
    ui.version_number_label->setText(version_qstr);

    widget->setWindowTitle("Rover Interface: Built on " + QString::fromUtf8(BUILD_TIME) );
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea

    string rover_name_msg = "<font color='white'>Rover: " + selected_rover_name + "</font>";
    QString rover_name_msg_qstr = QString::fromStdString(rover_name_msg);
    ui.rover_name->setText(rover_name_msg_qstr);

    // Setup QT message connections
<<<<<<< HEAD
    connect(this, SIGNAL(sendDiagsDataUpdate(QString, QString, QColor)), this, SLOT(receiveDiagsDataUpdate(QString, QString,QColor)));
=======
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
    connect(ui.rover_list, SIGNAL(currentItemChanged(QListWidgetItem*,QListWidgetItem*)), this, SLOT(currentRoverChangedEventHandler(QListWidgetItem*,QListWidgetItem*)));
    connect(ui.rover_list, SIGNAL(itemDoubleClicked(QListWidgetItem*)), this, SLOT(refocusKeyboardEventHandler()));
    connect(ui.rover_list, SIGNAL(itemClicked(QListWidgetItem*)), this, SLOT(refocusKeyboardEventHandler()));
    connect(ui.ekf_checkbox, SIGNAL(toggled(bool)), this, SLOT(EKFCheckboxToggledEventHandler(bool)));
    connect(ui.gps_checkbox, SIGNAL(toggled(bool)), this, SLOT(GPSCheckboxToggledEventHandler(bool)));
    connect(ui.encoder_checkbox, SIGNAL(toggled(bool)), this, SLOT(encoderCheckboxToggledEventHandler(bool)));
    connect(ui.autonomous_control_radio_button, SIGNAL(toggled(bool)), this, SLOT(autonomousRadioButtonEventHandler(bool)));
    connect(ui.joystick_control_radio_button, SIGNAL(toggled(bool)), this, SLOT(joystickRadioButtonEventHandler(bool)));
    connect(ui.all_autonomous_button, SIGNAL(pressed()), this, SLOT(allAutonomousButtonEventHandler()));
    connect(ui.all_stop_button, SIGNAL(pressed()), this, SLOT(allStopButtonEventHandler()));
    connect(ui.build_simulation_button, SIGNAL(pressed()), this, SLOT(buildSimulationButtonEventHandler()));
    connect(ui.clear_simulation_button, SIGNAL(pressed()), this, SLOT(clearSimulationButtonEventHandler()));
    connect(ui.visualize_simulation_button, SIGNAL(pressed()), this, SLOT(visualizeSimulationButtonEventHandler()));
<<<<<<< HEAD
    connect(ui.map_auto_radio_button, SIGNAL(toggled(bool)), this, SLOT(mapAutoRadioButtonEventHandler(bool)));
    connect(ui.map_manual_radio_button, SIGNAL(toggled(bool)), this, SLOT(mapManualRadioButtonEventHandler(bool)));
    connect(ui.map_popout_button, SIGNAL(pressed()), this, SLOT(mapPopoutButtonEventHandler()));
    connect(this, SIGNAL(allStopButtonSignal()), this, SLOT(allStopButtonEventHandler()));


    // Joystick output display - Drive
    connect(this, SIGNAL(joystickDriveForwardUpdate(double)), ui.joy_lcd_drive_forward, SLOT(display(double)));
    connect(this, SIGNAL(joystickDriveBackwardUpdate(double)), ui.joy_lcd_drive_back, SLOT(display(double)));
    connect(this, SIGNAL(joystickDriveLeftUpdate(double)), ui.joy_lcd_drive_left, SLOT(display(double)));
    connect(this, SIGNAL(joystickDriveRightUpdate(double)), ui.joy_lcd_drive_right, SLOT(display(double)));

    // Joystick output display - Gripper
    connect(this, SIGNAL(joystickGripperWristUpUpdate(double)), ui.joy_lcd_gripper_up, SLOT(display(double)));
    connect(this, SIGNAL(joystickGripperWristDownUpdate(double)), ui.joy_lcd_gripper_down, SLOT(display(double)));
    connect(this, SIGNAL(joystickGripperFingersCloseUpdate(double)), ui.joy_lcd_gripper_close, SLOT(display(double)));
    connect(this, SIGNAL(joystickGripperFingersOpenUpdate(double)), ui.joy_lcd_gripper_open, SLOT(display(double)));

    connect(this, SIGNAL(updateCurrentSimulationTimeLabel(QString)), ui.currentSimulationTimeLabel, SLOT(setText(QString)));
    connect(this, SIGNAL(updateObstacleCallCount(QString)), ui.perc_of_time_avoiding_obstacles, SLOT(setText(QString)));
    connect(this, SIGNAL(updateNumberOfTagsCollected(QString)), ui.num_targets_collected_label, SLOT(setText(QString)));
    connect(this, SIGNAL(updateNumberOfSatellites(QString)), ui.gps_numSV_label, SLOT(setText(QString)));
    connect(this, SIGNAL(sendInfoLogMessage(QString)), this, SLOT(receiveInfoLogMessage(QString)));
    connect(this, SIGNAL(sendDiagLogMessage(QString)), this, SLOT(receiveDiagLogMessage(QString)));
    connect(ui.custom_world_path_button, SIGNAL(pressed()), this, SLOT(customWorldButtonEventHandler()));
    connect(ui.custom_distribution_radio_button, SIGNAL(toggled(bool)), this, SLOT(customWorldRadioButtonEventHandler(bool)));
    connect(ui.override_num_rovers_checkbox, SIGNAL(toggled(bool)), this, SLOT(overrideNumRoversCheckboxToggledEventHandler(bool)));

    // Receive log messages from contained frames
    connect(ui.map_frame, SIGNAL(sendInfoLogMessage(QString)), this, SLOT(receiveInfoLogMessage(QString)));

    // Add the checkbox handler so we can process events. We have to listen for itemChange events since
    // we don't have a real chackbox with toggle events
    connect(ui.map_selection_list, SIGNAL(itemChanged(QListWidgetItem*)), this, SLOT(mapSelectionListItemChangedHandler(QListWidgetItem*)));
=======
    connect(this, SIGNAL(joystickForwardUpdate(double)), ui.joy_lcd_forward, SLOT(display(double)));
    connect(this, SIGNAL(joystickBackUpdate(double)), ui.joy_lcd_back, SLOT(display(double)));
    connect(this, SIGNAL(joystickLeftUpdate(double)), ui.joy_lcd_left, SLOT(display(double)));
    connect(this, SIGNAL(joystickRightUpdate(double)), ui.joy_lcd_right, SLOT(display(double)));
    connect(this, SIGNAL(updateObstacleCallCount(QString)), ui.perc_of_time_avoiding_obstacles, SLOT(setText(QString)));
    connect(this, SIGNAL(updateLog(QString)), this, SLOT(displayLogMessage(QString)));
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea

    // Create a subscriber to listen for joystick events
    joystick_subscriber = nh.subscribe("/joy", 1000, &RoverGUIPlugin::joyEventHandler, this);

<<<<<<< HEAD
    emit sendInfoLogMessage("Searching for rovers...");
=======
    displayLogMessage("Searching for rovers...");
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea

    // Add discovered rovers to the GUI list
    rover_poll_timer = new QTimer(this);
    connect(rover_poll_timer, SIGNAL(timeout()), this, SLOT(pollRoversTimerEventHandler()));
    rover_poll_timer->start(5000);

    // Setup the initial display parameters for the map
<<<<<<< HEAD
    ui.map_frame->setMapData(map_data);
    ui.map_frame->createPopoutWindow(map_data); // This has to happen before the display radio buttons are set
=======
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
    ui.map_frame->setDisplayGPSData(ui.gps_checkbox->isChecked());
    ui.map_frame->setDisplayEncoderData(ui.encoder_checkbox->isChecked());
    ui.map_frame->setDisplayEKFData(ui.ekf_checkbox->isChecked());

<<<<<<< HEAD
    ui.joystick_frame->setHidden(false);

    ui.custom_world_path_button->setEnabled(true);
    ui.custom_world_path_button->setStyleSheet("color: white; border:1px solid white;");

    // Make the custom rover number combo box look greyed out to begin with
    ui.custom_num_rovers_combobox->setStyleSheet("color: grey; border:2px solid grey;");
=======
    ui.joystick_frame->setHidden(true);
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea

    ui.tab_widget->setCurrentIndex(0);

    ui.texture_combobox->setItemData(0, Qt::white, Qt::TextColorRole);

    ui.visualize_simulation_button->setEnabled(false);
    ui.clear_simulation_button->setEnabled(false);
    ui.all_autonomous_button->setEnabled(false);
    ui.all_stop_button->setEnabled(false);

    ui.visualize_simulation_button->setStyleSheet("color: grey; border:2px solid grey;");
    ui.clear_simulation_button->setStyleSheet("color: grey; border:2px solid grey;");
    ui.all_autonomous_button->setStyleSheet("color: grey; border:2px solid grey;");
    ui.all_stop_button->setStyleSheet("color: grey; border:2px solid grey;");

    //QString return_msg = startROSJoyNode();
    //displayLogMessage(return_msg);
<<<<<<< HEAD

    info_log_subscriber = nh.subscribe("/infoLog", 10, &RoverGUIPlugin::infoLogMessageEventHandler, this);
    diag_log_subscriber = nh.subscribe("/diagsLog", 10, &RoverGUIPlugin::diagLogMessageEventHandler, this);

    emit updateNumberOfSatellites("<font color='white'>Number of GPS Satellites: ---</font>");
=======
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
  }

  void RoverGUIPlugin::shutdownPlugin()
  {
<<<<<<< HEAD
    map_data->clear(); // Clear the map and stop drawing before the map_frame is destroyed
    ui.map_frame->clear();
=======
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
    clearSimulationButtonEventHandler();
    rover_poll_timer->stop();
    stopROSJoyNode();
    ros::shutdown();
  }

void RoverGUIPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
}

void RoverGUIPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
}

<<<<<<< HEAD

// Receives messages from the ROS joystick driver and used them to articulate the gripper and drive the rover.
void RoverGUIPlugin::joyEventHandler(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
     // Give the array values some helpful names:
    int left_stick_x_axis = 0; // Gripper fingers close and open
    int left_stick_y_axis = 1; // Gripper wrist up and down

    int right_stick_x_axis = 3; // Turn left and right
    int right_stick_y_axis = 4; // Drive forward and backward

    // Note: joystick stick axis output value are between -1 and 1

     if (joystick_publisher)
        {
         // Handle drive commands - BEGIN

        //Set the gui values. Filter values to be large enough to move the physical rover.
        if (joy_msg->axes[right_stick_y_axis] >= 0.1)
        {
            emit joystickDriveForwardUpdate(joy_msg->axes[right_stick_y_axis]);
        }
        if (joy_msg->axes[right_stick_y_axis] <= -0.1)
        {
            emit joystickDriveBackwardUpdate(-joy_msg->axes[right_stick_y_axis]);
        }
        //If value is too small, display 0.
        if (abs(joy_msg->axes[right_stick_y_axis]) < 0.1)
        {
            emit joystickDriveForwardUpdate(0);
            emit joystickDriveBackwardUpdate(0);
        }

        if (joy_msg->axes[right_stick_x_axis] >= 0.1)
        {
	  emit joystickDriveLeftUpdate(joy_msg->axes[right_stick_x_axis]);
        }
        if (joy_msg->axes[right_stick_x_axis] <= -0.1)
        {
	  emit joystickDriveRightUpdate(-joy_msg->axes[right_stick_x_axis]);
        }
        //If value is too small, display 0.
        if (abs(joy_msg->axes[right_stick_x_axis]) < 0.1)
        {
            emit joystickDriveLeftUpdate(0);
            emit joystickDriveRightUpdate(0);
        }

        // Handle drive commands - END

        // Handle gripper commands - BEGIN

        // The joystick output is a 1D vector since it has a direction (-/+) and a magnitude.
        // This vector is processed by the JoystickGripperInterface to produce gripper angle commands
        float wristCommandVector = joy_msg->axes[left_stick_y_axis];
        float fingerCommandVector = joy_msg->axes[left_stick_x_axis];

        // These if statements just determine which GUI element to update.
        if (wristCommandVector >= 0.1)
        {
            emit joystickGripperWristUpUpdate(wristCommandVector);
        }
        if (wristCommandVector <= -0.1)
        {
            emit joystickGripperWristDownUpdate(-wristCommandVector);
        }

        //If value is too small, display 0
        if (abs(wristCommandVector) < 0.1)
        {
            emit joystickGripperWristUpUpdate(0);
            emit joystickGripperWristDownUpdate(0);
        }

        if (fingerCommandVector >= 0.1)
        {
            emit joystickGripperFingersCloseUpdate(fingerCommandVector);
        }

        if (fingerCommandVector <= -0.1)
        {
            emit joystickGripperFingersOpenUpdate(-fingerCommandVector);
        }

        //If value is too small, display 0
        if (abs(fingerCommandVector) < 0.1)
        {
            emit joystickGripperFingersCloseUpdate(0);
            emit joystickGripperFingersOpenUpdate(0);
        }

        // Use the joystick output to generate ROS gripper commands
        // Lock this section so the interface isnt recreated while in use

        if (joystickGripperInterface)
        {
            try {
                joystickGripperInterface->moveWrist(wristCommandVector);
                joystickGripperInterface->moveFingers(fingerCommandVector);
            } catch (JoystickGripperInterfaceNotReadyException e) {
                emit sendInfoLogMessage("Tried to use the joystick gripper interface before it was ready.");
            }

        }
        else
        {
            emit sendInfoLogMessage("Error: joystickGripperInterface has not been instantiated.");
        }

        // Handle gripper commands - END


        joystick_publisher.publish(joy_msg);
    }
=======
void RoverGUIPlugin::joyEventHandler(const sensor_msgs::Joy::ConstPtr& joy_msg)
{

    //Set the gui values. Filter values to be large enough to move the physical rover.
    if (joy_msg->axes[4] >= 0.1)
    {
        emit joystickForwardUpdate(joy_msg->axes[4]);
    }
    if (joy_msg->axes[4] <= -0.1)
    {
        emit joystickBackUpdate(-joy_msg->axes[4]);
    }
    //If value is too small, display 0.
    if (abs(joy_msg->axes[4]) < 0.1)
    {
        emit joystickForwardUpdate(0);
        emit joystickBackUpdate(0);
    }

    if (joy_msg->axes[3] >= 0.1)
    {
        emit joystickLeftUpdate(joy_msg->axes[3]);
    }
    if (joy_msg->axes[3] <= -0.1)
    {
        emit joystickRightUpdate(-joy_msg->axes[3]);
    }
    //If value is too small, display 0.
    if (abs(joy_msg->axes[3]) < 0.1)
    {
        emit joystickLeftUpdate(0);
        emit joystickRightUpdate(0);
    }

// Magic axis values in the code below were taken the rover_driver_rqt_motor code /joystick output for default linear and angular velocities.
// Magic indicies are taken from rover_motor.cpp.
// This way the code is consistent with the existing GUI joystick.
// A better way would be to standardize a manual movement control interface and requre all input mechanisms to take input from the user
// and repackage te information according to the interface spec.
    geometry_msgs::Twist standardized_joy_msg;

    if (abs(joy_msg->axes[4]) >= 0.1)
    {
      standardized_joy_msg.linear.x = joy_msg->axes[4];
    }

    if (abs(joy_msg->axes[3]) >= 0.1)
    {
      standardized_joy_msg.angular.z = joy_msg->axes[3];
    }

    joystick_publisher.publish(standardized_joy_msg);

>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
}

void RoverGUIPlugin::EKFEventHandler(const ros::MessageEvent<const nav_msgs::Odometry> &event)
{
    const std::string& publisher_name = event.getPublisherName();
    const ros::M_string& header = event.getConnectionHeader();
    ros::Time receipt_time = event.getReceiptTime();

    const boost::shared_ptr<const nav_msgs::Odometry> msg = event.getMessage();

    float x = msg->pose.pose.position.x;
    float y = msg->pose.pose.position.y;

    QString x_str; x_str.setNum(x);
    QString y_str; y_str.setNum(y);
<<<<<<< HEAD
    // Extract rover name from the message source. Publisher is in the format /*rover_name*_MAP
    size_t found = publisher_name.find("_MAP");
=======
    // Extract rover name from the message source. Publisher is in the format /*rover_name*_GPS
    size_t found = publisher_name.find("_EKF");
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
    string rover_name = publisher_name.substr(1,found-1);

    // Store map info for the appropriate rover name
    ui.map_frame->addToEKFRoverPath(rover_name, x, y);
}


void RoverGUIPlugin::encoderEventHandler(const ros::MessageEvent<const nav_msgs::Odometry> &event)
{
    const std::string& publisher_name = event.getPublisherName();
    const ros::M_string& header = event.getConnectionHeader();
    ros::Time receipt_time = event.getReceiptTime();

    string topic = header.at("topic");

    const boost::shared_ptr<const nav_msgs::Odometry> msg = event.getMessage();
    float x = msg->pose.pose.position.x;
    float y = msg->pose.pose.position.y;

    QString x_str; x_str.setNum(x);
    QString y_str; y_str.setNum(y);

    // Extract rover name from the message source. Get the topic name from the event header. Can't use publisher_name here because it is just /gazebo.
<<<<<<< HEAD
    size_t found = topic.find("/odom/filtered");
=======
    size_t found = topic.find("/odom");
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
    string rover_name = topic.substr(1,found-1);

    // Store map info for the appropriate rover name
   ui.map_frame->addToEncoderRoverPath(rover_name, x, y);
}


void RoverGUIPlugin::GPSEventHandler(const ros::MessageEvent<const nav_msgs::Odometry> &event)
{
    const std::string& publisher_name = event.getPublisherName();
    const ros::M_string& header = event.getConnectionHeader();
    ros::Time receipt_time = event.getReceiptTime();

    const boost::shared_ptr<const nav_msgs::Odometry> msg = event.getMessage();
    float x = msg->pose.pose.position.x;
    float y = msg->pose.pose.position.y;

    QString x_str; x_str.setNum(x);
    QString y_str; y_str.setNum(y);

    // Extract rover name from the message source. Publisher is in the format /*rover_name*_NAVSAT
    size_t found = publisher_name.find("_NAVSAT");
    string rover_name = publisher_name.substr(1,found-1);

    // Store map info for the appropriate rover name
    ui.map_frame->addToGPSRoverPath(rover_name, x, y);
}

<<<<<<< HEAD
void RoverGUIPlugin::GPSNavSolutionEventHandler(const ros::MessageEvent<const ublox_msgs::NavSOL> &event) {
    const boost::shared_ptr<const ublox_msgs::NavSOL> msg = event.getMessage();

    // Extract rover name from the message source. Publisher is in the format /*rover_name*_UBLOX
    size_t found = event.getPublisherName().find("_UBLOX");
    QString rover_name = event.getPublisherName().substr(1,found-1).c_str();

    // Update the number of sattellites detected for the specified rover
    rover_numSV_state[rover_name.toStdString()] = msg.get()->numSV;

    // only update the label if a rover is selected by the user in the GUI
    // and the number of detected satellites is > 0
    if (selected_rover_name.compare("") != 0 && msg.get()->numSV > 0) {
        // Update the label in the GUI with the selected rover's information
        QString newLabelText = "Number of GPS Satellites: " + QString::number(rover_numSV_state[selected_rover_name]);
        emit updateNumberOfSatellites("<font color='white'>" + newLabelText + "</font>");
    } else {
        emit updateNumberOfSatellites("<font color='white'>Number of GPS Satellites: ---</font>");
    }
}

=======
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
 void RoverGUIPlugin::cameraEventHandler(const sensor_msgs::ImageConstPtr& image)
 {
     cv_bridge::CvImagePtr cv_image_ptr;

     try
     {
        cv_image_ptr = cv_bridge::toCvCopy(image);
     }
     catch (cv_bridge::Exception &e)
     {
         ROS_ERROR("In rover_gui_plugin.cpp: cv_bridge exception: %s", e.what());
     }

     int image_cols = cv_image_ptr->image.cols;
     int image_rows = cv_image_ptr->image.rows;
     int image_step = cv_image_ptr->image.step;

     ostringstream cols_stream, rows_stream;
     cols_stream << image_cols;
     rows_stream << image_rows;

     // For debugging
     //QString debug_msg = "Received image ";
     //debug_msg += "(" + QString::number(image_rows) + " x " + QString::number(image_cols) +")";
     //cout << debug_msg.toStdString() << endl;

    // ROS_INFO_STREAM("Image received Size:" + rows_stream.str() + "x" + cols_stream.str());

     // Create QImage to hold the image
     //const uchar* image_buffer = (const uchar*)cv_image_ptr->image.data; // extract the raw data
     QImage qimg(&(image->data[0]), image_cols, image_rows, image_step, QImage::Format_RGB888);
     qimg = qimg.rgbSwapped(); // Convert from RGB to BGR which is the output format for the rovers.
     ui.camera_frame->setImage(qimg);
 }

set<string> RoverGUIPlugin::findConnectedRovers()
{
    set<string> rovers;

    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);

    stringstream ss;

   for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++)
    {
        const ros::master::TopicInfo& info = *it;

        string rover_name;

        std::size_t found = info.name.find("/status");
          if (found!=std::string::npos)
          {
            rover_name = info.name.substr(1,found-1);

            found = rover_name.find("/"); // Eliminate potential names with / in them
            if (found==std::string::npos)
            {
                rovers.insert(rover_name);
            }
        }
    }

    return rovers;
}

<<<<<<< HEAD
// Receives and stores the status update messages from rovers
void RoverGUIPlugin::statusEventHandler(const ros::MessageEvent<std_msgs::String const> &event)
{
    const ros::M_string& header = event.getConnectionHeader();
    ros::Time receipt_time = event.getReceiptTime();

    // Extract rover name from the message source

    // This method is used rather than reading the publisher name to accomodate teams that changed the node name.
    string topic = header.at("topic");
    size_t found = topic.find("/status");
    string rover_name = topic.substr(1,found-1);

    const std_msgs::StringConstPtr& msg = event.getMessage();

    string status = msg->data;

    RoverStatus rover_status;
    rover_status.status_msg = status;
    rover_status.timestamp = receipt_time;

    rover_statuses[rover_name] = rover_status;
=======
void RoverGUIPlugin::targetPickUpEventHandler(const ros::MessageEvent<const sensor_msgs::Image> &event)
{
    const std::string& publisher_name = event.getPublisherName();
    const ros::M_string& header = event.getConnectionHeader();
    ros::Time receipt_time = event.getReceiptTime();
    
    const sensor_msgs::ImageConstPtr& image = event.getMessage();

    // Extract rover name from the message source
    string topic = header.at("topic");
    size_t found = topic.find("/targetPickUpImage");
    string rover_name = topic.substr(1,found-1);

    int targetID = targetDetect(image);
    
    //Check all robots to ensure that no one is already holding the target
    bool targetPreviouslyCollected = false;
	for (map<string,int>::iterator it=targetsPickedUp.begin(); it!=targetsPickedUp.end(); ++it) {
		if (it->second == targetID) {
			targetPreviouslyCollected = true;
			break;
		}
	}
	
    if((targetID < 0) || (targetID == collectionZoneID) || targetPreviouslyCollected) {
        // No valid target was found in the image, or the target was the collection zone ID, or the target was already picked up by another robot
        
        //Publish -1 to alert robot of failed drop off event
        std_msgs::Int16 targetIDMsg;
        targetIDMsg.data = -1;
        targetPickUpPublisher[rover_name].publish(targetIDMsg);
    }
    else {
		//Record target ID according to the rover that reported it
        targetsPickedUp[rover_name] = targetID;
        emit updateLog("Resource " + QString::number(targetID) + " picked up by " + QString::fromStdString(rover_name));
        ui.num_targets_detected_label->setText(QString("<font color='white'>")+QString::number(targetsPickedUp.size())+QString("</font>"));
        
        //Publish target ID
        std_msgs::Int16 targetIDMsg;
        targetIDMsg.data = targetID;
        targetPickUpPublisher[rover_name].publish(targetIDMsg);
    }
}

void RoverGUIPlugin::targetDropOffEventHandler(const ros::MessageEvent<const sensor_msgs::Image> &event)
{
    const std::string& publisher_name = event.getPublisherName();
    const ros::M_string& header = event.getConnectionHeader();
    ros::Time receipt_time = event.getReceiptTime();

    const sensor_msgs::ImageConstPtr& image = event.getMessage();

    // Extract rover name from the message source
    string topic = header.at("topic");
    size_t found = topic.find("/targetDropOffImage");
    string rover_name = topic.substr(1,found-1);

    int targetID = targetDetect(image);

    if(targetID != collectionZoneID) {
        // This target does not match the official collection zone ID
    }
    else {
		//Use try-catch here in case a rover reports the collection zone ID without ever having picked up a target
        try {
			//Add target ID to list of dropped off targets
            targetsDroppedOff[targetsPickedUp.at(rover_name)] = true;
            emit updateLog("Resource " + QString::number(targetsPickedUp.at(rover_name)) + " dropped off by " + QString::fromStdString(rover_name));
            ui.num_targets_collected_label->setText(QString("<font color='white'>")+QString::number(targetsDroppedOff.size())+QString("</font>"));
            targetsPickedUp.erase(rover_name);
            ui.num_targets_detected_label->setText(QString("<font color='white'>")+QString::number(targetsPickedUp.size())+QString("</font>"));
            
            //Publish target ID (should always be equal to 256)
			std_msgs::Int16 targetIDMsg;
			targetIDMsg.data = targetID;
			targetDropOffPublisher[rover_name].publish(targetIDMsg);
        }
        catch(const std::out_of_range& oor) {
            emit updateLog(QString::fromStdString(rover_name) + " attempted a drop off but was not carrying a target");
            
            //Publish -1 to alert robot of failed drop off event
            std_msgs::Int16 targetIDMsg;
			targetIDMsg.data = -1;
			targetDropOffPublisher[rover_name].publish(targetIDMsg);
        }
    }
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
}

// Counts the number of obstacle avoidance calls
void RoverGUIPlugin::obstacleEventHandler(const ros::MessageEvent<const std_msgs::UInt8> &event)
{
    const std::string& publisher_name = event.getPublisherName();
    const ros::M_string& header = event.getConnectionHeader();
    ros::Time receipt_time = event.getReceiptTime();

    const std_msgs::UInt8ConstPtr& msg = event.getMessage();

    //QString displ = QString("Target number ") + QString::number(msg->data) + QString(" found.");

    // 0 for no obstacle, 1 for right side obstacle, and 2 for left side obsticle
    int code = msg->data;

    if (code != 0)
    {
        emit updateObstacleCallCount("<font color='white'>"+QString::number(++obstacle_call_count)+"</font>");
    }
}

<<<<<<< HEAD
// Takes the published score value from the ScorePlugin and updates the GUI
void RoverGUIPlugin::scoreEventHandler(const ros::MessageEvent<const std_msgs::String> &event) {
    const std::string& publisher_name = event.getPublisherName();
    const ros::M_string& header = event.getConnectionHeader();
    ros::Time receipt_time = event.getReceiptTime();

    const std_msgs::StringConstPtr& msg = event.getMessage();
    std::string tags_collected = msg->data;

    emit updateNumberOfTagsCollected("<font color='white'>"+QString::fromStdString(tags_collected)+"</font>");
}

void RoverGUIPlugin::simulationTimerEventHandler(const rosgraph_msgs::Clock& msg) {

    current_simulated_time_in_seconds = msg.clock.toSec();

    bool updateTimeLabel = ((current_simulated_time_in_seconds - last_current_time_update_in_seconds) >= 1.0) ? (true) : (false);

    // only update the current time once per second; faster update rates make the GUI unstable
    // and in the worst cases it will hang and/or crash
    if (updateTimeLabel == true) {
        emit updateCurrentSimulationTimeLabel("<font color='white'>" +
                                              QString::number(getHours(current_simulated_time_in_seconds)) + " hours, " +
                                              QString::number(getMinutes(current_simulated_time_in_seconds)) + " minutes, " +
                                              QString::number(floor(getSeconds(current_simulated_time_in_seconds))) + " seconds</font>");
        last_current_time_update_in_seconds = current_simulated_time_in_seconds;
    }

    // this catches the case when the /clock timer is not running
    // AKA: when we are not running a simulation
    if (current_simulated_time_in_seconds <= 0.0) {
        return;
    }

    if (is_timer_on == true) {
        if (current_simulated_time_in_seconds >= timer_stop_time_in_seconds) {
            is_timer_on = false;
            emit allStopButtonSignal();
            emit sendInfoLogMessage("\nSimulation timer complete at: " +
                                    QString::number(getHours(current_simulated_time_in_seconds)) + " hours, " +
                                    QString::number(getMinutes(current_simulated_time_in_seconds)) + " minutes, " +
                                    QString::number(getSeconds(current_simulated_time_in_seconds)) + " seconds\n");
        }
    }
}

void RoverGUIPlugin::currentRoverChangedEventHandler(QListWidgetItem *current, QListWidgetItem *previous)
{
    emit sendInfoLogMessage("Selcted Rover Changed");

    if (!current) return; // Check to make sure the current selection isn't null

    // Extract rover name
    string rover_name_and_status = current->text().toStdString();

    // Rover names start at the begining of the rover name and status string and end at the first space
    size_t rover_name_length = rover_name_and_status.find_first_of(" ");
    string ui_rover_name = rover_name_and_status.substr(0, rover_name_length);

    selected_rover_name = ui_rover_name;

=======
void RoverGUIPlugin::currentRoverChangedEventHandler(QListWidgetItem *current, QListWidgetItem *previous)
{
    displayLogMessage("Selcted Rover Changed");

    if (!current) return; // Check to make sure the current selection isn't null

    selected_rover_name = current->text().toStdString();
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
    string rover_name_msg = "<font color='white'>Rover: " + selected_rover_name + "</font>";
    QString rover_name_msg_qstr = QString::fromStdString(rover_name_msg);
    ui.rover_name->setText(rover_name_msg_qstr);

<<<<<<< HEAD
    emit sendInfoLogMessage(QString("Selected rover: ") + QString::fromStdString(selected_rover_name));
=======
    displayLogMessage(QString("Selected rover: ") + QString::fromStdString(selected_rover_name));
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea

    // Attempt to read the simulation model xml file if it exists. If it does not exist assume this is a physical rover.
    const char *name = "GAZEBO_MODEL_PATH";
    char *model_root_cstr;
    model_root_cstr = getenv(name);
    QString model_root(model_root_cstr);

    QString model_path = model_root+"/"+QString::fromStdString(selected_rover_name)+"/model.sdf";

    readRoverModelXML(model_path);
<<<<<<< HEAD
    
    //Set up subscribers
    image_transport::ImageTransport it(nh);
    camera_subscriber = it.subscribe("/"+selected_rover_name+"/targets/image", 1, &RoverGUIPlugin::cameraEventHandler, this, image_transport::TransportHints("theora"));
    imu_subscriber = nh.subscribe("/"+selected_rover_name+"/imu", 10, &RoverGUIPlugin::IMUEventHandler, this);
    us_center_subscriber = nh.subscribe("/"+selected_rover_name+"/sonarCenter", 10, &RoverGUIPlugin::centerUSEventHandler, this);
    us_left_subscriber = nh.subscribe("/"+selected_rover_name+"/sonarLeft", 10, &RoverGUIPlugin::leftUSEventHandler, this);
    us_right_subscriber = nh.subscribe("/"+selected_rover_name+"/sonarRight", 10, &RoverGUIPlugin::rightUSEventHandler, this);

    emit sendInfoLogMessage(QString("Displaying map for ")+QString::fromStdString(selected_rover_name));

    // Add to the rover map.
    QListWidgetItem* map_selection_item = ui.map_selection_list->item(ui.rover_list->row(current));
    map_selection_item->setCheckState(Qt::Checked);
=======

    setupPublishers();
    setupSubscribers();

    displayLogMessage(QString("Displaying map for ")+QString::fromStdString(selected_rover_name));
    ui.map_frame->setRoverMapToDisplay(selected_rover_name);

    std::map<string, int>::iterator it = rover_control_state.find(selected_rover_name);
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea

    // No entry for this rover name
    if ( 0 == rover_control_state.count(selected_rover_name) )
    {
        // Default to joystick
        ui.joystick_control_radio_button->setChecked(true);
        ui.autonomous_control_radio_button->setChecked(false);
        joystickRadioButtonEventHandler(true); // Manually trigger the joystick selected event
        rover_control_state[selected_rover_name]=1;
<<<<<<< HEAD
        emit sendInfoLogMessage("New rover selected");
    }
    else
    {
        int control_state = rover_control_state.find(selected_rover_name)->second;
=======
        displayLogMessage("New rover selected");
    }
    else
    {
        int control_state = it->second;
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea

        switch (control_state)
        {
        case 1: // manual
            ui.joystick_control_radio_button->setChecked(true);
            ui.autonomous_control_radio_button->setChecked(false);
            ui.joystick_frame->setHidden(false);
            joystickRadioButtonEventHandler(true); // Manually trigger the joystick selected event
            break;
        case 2: // autonomous
            ui.joystick_control_radio_button->setChecked(false);
            ui.autonomous_control_radio_button->setChecked(true);
            ui.joystick_frame->setHidden(true);
<<<<<<< HEAD
            autonomousRadioButtonEventHandler(true); // Manually trigger the autonomous selected event
            break;
        default:
            emit sendInfoLogMessage("Unknown control state: "+QString::number(control_state));
        }

        emit sendInfoLogMessage("Existing rover selected");
    }

    // only update the number of satellites if a valid rover name has been selected
    if (selected_rover_name.compare("") != 0 && rover_numSV_state[selected_rover_name] > 0) {
        QString newLabelText = "Number of GPS Satellites: " + QString::number(rover_numSV_state[selected_rover_name]);
        emit updateNumberOfSatellites("<font color='white'>" + newLabelText + "</font>");
    } else {
        emit updateNumberOfSatellites("<font color='white'>Number of GPS Satellites: ---</font>");
    }
=======
            break;
        default:
            displayLogMessage("Unknown control state: "+QString::number(control_state));
        }

        displayLogMessage("Existing rover selected");
    }

    // Clear map
    // ui.map_frame->clearMap();
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea

    // Enable control mode radio group now that a rover has been selected
    ui.autonomous_control_radio_button->setEnabled(true);
    ui.joystick_control_radio_button->setEnabled(true);
}

void RoverGUIPlugin::pollRoversTimerEventHandler()
{
<<<<<<< HEAD
    // Returns rovers that have created a status topic
    set<string>new_rover_names = findConnectedRovers();

=======
    set<string>new_rover_names = findConnectedRovers();


>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
    std::set<string> orphaned_rover_names;

    // Calculate which of the old rover names are not in the new list of rovers then clear their maps and control states.
    std::set_difference(rover_names.begin(), rover_names.end(), new_rover_names.begin(), new_rover_names.end(),
        std::inserter(orphaned_rover_names, orphaned_rover_names.end()));

    for (set<string>::iterator it = orphaned_rover_names.begin(); it != orphaned_rover_names.end(); ++it)
    {
<<<<<<< HEAD
        emit sendInfoLogMessage(QString("Clearing interface data for disconnected rover ") + QString::fromStdString(*it));
        map_data->clear(*it);
        ui.map_frame->clear(*it);
        rover_control_state.erase(*it); // Remove the control state for orphaned rovers
        rover_numSV_state.erase(*it);
        rover_statuses.erase(*it);

        // If the currently selected rover disconnected, shutdown its subscribers and publishers
        if (it->compare(selected_rover_name) == 0)
        {
            camera_subscriber.shutdown();
            imu_subscriber.shutdown();
            us_center_subscriber.shutdown();
            us_left_subscriber.shutdown();
            us_right_subscriber.shutdown();
            joystick_publisher.shutdown();

            //Reset selected rover name to empty string
            selected_rover_name = "";
        }

        // For the other rovers that disconnected...

        // Shutdown the subscribers
        status_subscribers[*it].shutdown();
        encoder_subscribers[*it].shutdown();
        gps_subscribers[*it].shutdown();
        gps_nav_solution_subscribers[*it].shutdown();
        ekf_subscribers[*it].shutdown();
        rover_diagnostic_subscribers[*it].shutdown();

        // Delete the subscribers
        status_subscribers.erase(*it);
        encoder_subscribers.erase(*it);
        gps_subscribers.erase(*it);
        gps_nav_solution_subscribers.erase(*it);
        ekf_subscribers.erase(*it);
        rover_diagnostic_subscribers.erase(*it);
        
        // Shudown Publishers
        control_mode_publishers[*it].shutdown();

        // Delete Publishers
        control_mode_publishers.erase(*it);
=======
        displayLogMessage(QString("Clearing interface data for disconnected rover ") + QString::fromStdString(*it));
        ui.map_frame->clearMap(*it);
        rover_control_state.erase(*it); // Remove the control state for orphaned rovers
        
        // If the currently selected rover disconnected shutdown its subscribers and publishers
        if (it->compare(selected_rover_name) == 0)
        {
          //  displayLogMessage("Shutting down and deleting "+QString::fromStdString(*it)+"'s camera subscriber.");
			camera_subscriber.shutdown();
          //  displayLogMessage("Shutting down and deleting "+QString::fromStdString(*it)+"'s IMU subscriber.");
            imu_subscriber.shutdown();
          //  displayLogMessage("Shutting down and deleting "+QString::fromStdString(*it)+"'s central sonar subscriber.");
            us_center_subscriber.shutdown();
          //  displayLogMessage("Shutting down and deleting "+QString::fromStdString(*it)+"'s left sonar subscriber.");
            us_left_subscriber.shutdown();
          //  displayLogMessage("Shutting down and deleting "+QString::fromStdString(*it)+"'s right sonar subscriber.");
			us_right_subscriber.shutdown();

            joystick_publisher.shutdown();
        }

       // For the other rovers that disconnected...

        // Shutdown the subscribers
        encoder_subscribers[*it].shutdown();
        gps_subscribers[*it].shutdown();
        ekf_subscribers[*it].shutdown();

        // Delete the subscribers
        encoder_subscribers.erase(*it);
        //displayLogMessage("Shutting down and deleting "+QString::fromStdString(*it)+"'s encoder subscriber ("+QString::number(encoder_subscribers.size())+" remaining)");

        gps_subscribers.erase(*it);
        //displayLogMessage("Shutting down and deleting "+QString::fromStdString(*it)+"'s GPS subscriber ("+QString::number(gps_subscribers.size())+" remaining)");

        ekf_subscribers.erase(*it);
        //displayLogMessage("Shutting down and deleting "+QString::fromStdString(*it)+"'s EKF subscriber ("+QString::number(ekf_subscribers.size())+" remaining)");

        // Shudown Publishers
        control_mode_publishers[*it].shutdown();
        targetPickUpPublisher[*it].shutdown();
        targetDropOffPublisher[*it].shutdown();

        // Delete Publishers
        control_mode_publishers.erase(*it);
        //displayLogMessage("Shutting down and deleting "+QString::fromStdString(*it)+"'s control mode publisher ("+QString::number(control_mode_publishers.size())+" remaining)");

        targetPickUpPublisher.erase(*it);
        //displayLogMessage("Shutting down and deleting "+QString::fromStdString(*it)+"'s \"pick up\" publisher ("+QString::number(targetPickUpPublisher.size())+" remaining)");

        targetDropOffPublisher.erase(*it);
        //displayLogMessage("Shutting down and deleting "+QString::fromStdString(*it)+"'s \"drop off\" publisher ("+QString::number(targetDropOffPublisher.size())+" remaining)");
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
    }

    // Wait for a rover to connect
    if (new_rover_names.empty())
    {
        //displayLogMessage("Waiting for rover to connect...");
        selected_rover_name = "";
        rover_control_state.clear();
<<<<<<< HEAD
        rover_numSV_state.clear();
        rover_names.clear();        
        ui.rover_list->clearSelection();
        ui.rover_list->clear();
        ui.rover_diags_list->clear();
        ui.map_selection_list->clear();
=======
        rover_names.clear();        
        ui.rover_list->clearSelection();
        ui.rover_list->clear();
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea

        // Disable control mode group since no rovers are connected
        ui.autonomous_control_radio_button->setEnabled(false);
        ui.joystick_control_radio_button->setEnabled(false);
        ui.all_autonomous_button->setEnabled(false);
        ui.all_stop_button->setEnabled(false);
        ui.all_autonomous_button->setStyleSheet("color: grey; border:2px solid grey;");
        ui.all_stop_button->setStyleSheet("color: grey; border:2px solid grey;");
<<<<<<< HEAD

    }
    else if (new_rover_names == rover_names)
    {

        // Just update the statuses in ui rover list
        for(int row = 0; row < ui.rover_list->count(); row++)
        {
            QListWidgetItem *item = ui.rover_list->item(row);

            // Extract rover name
            string rover_name_and_status = item->text().toStdString();

            // Rover names start at the begining of the rover name and status string and end at the first space
            size_t rover_name_length = rover_name_and_status.find_first_of(" ");
            string ui_rover_name = rover_name_and_status.substr(0, rover_name_length);

            // Get current status

            RoverStatus updated_rover_status;
            // Build new ui rover list string
            try
            {
                updated_rover_status = rover_statuses.at(ui_rover_name);
            }
            catch (std::out_of_range& e)
            {
                emit sendInfoLogMessage("Error: No status entry for rover " + QString::fromStdString(ui_rover_name));
            }


            // Build new ui rover list string
            QString updated_rover_name_and_status = QString::fromStdString(ui_rover_name)
                                                    + " ("
                                                    + QString::fromStdString(updated_rover_status.status_msg)
                                                    + ")";

            // Update the UI
            item->setText(updated_rover_name_and_status);
        }
    }
    else
    {
    rover_names = new_rover_names;
    
    emit sendInfoLogMessage("List of connected rovers has changed");
    selected_rover_name = "";
    ui.rover_list->clearSelection();
    ui.rover_list->clear();

    // Also clear the rover diagnostics list
    ui.rover_diags_list->clear();
    ui.map_selection_list->clear();
=======
        return;
    }

    if (new_rover_names == rover_names)
    {
        return;
    }

    rover_names = new_rover_names;
    
    displayLogMessage("List of connected rovers has changed");
    selected_rover_name = "";
    ui.rover_list->clearSelection();
    ui.rover_list->clear();
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
    
    //Enable all autonomous button
    ui.all_autonomous_button->setEnabled(true);
    ui.all_autonomous_button->setStyleSheet("color: white; border:2px solid white;");

<<<<<<< HEAD
    // This code is from above. Consider moving into a function or restructuring
    for(set<string>::const_iterator i = rover_names.begin(); i != rover_names.end(); ++i)
    {
        //Set up publishers
        control_mode_publishers[*i]=nh.advertise<std_msgs::UInt8>("/"+*i+"/mode", 10, true); // last argument sets latch to true

        //Set up subscribers
        status_subscribers[*i] = nh.subscribe("/"+*i+"/status", 10, &RoverGUIPlugin::statusEventHandler, this);
        obstacle_subscribers[*i] = nh.subscribe("/"+*i+"/obstacle", 10, &RoverGUIPlugin::obstacleEventHandler, this);
        encoder_subscribers[*i] = nh.subscribe("/"+*i+"/odom/filtered", 10, &RoverGUIPlugin::encoderEventHandler, this);
        ekf_subscribers[*i] = nh.subscribe("/"+*i+"/odom/ekf", 10, &RoverGUIPlugin::EKFEventHandler, this);
        gps_subscribers[*i] = nh.subscribe("/"+*i+"/odom/navsat", 10, &RoverGUIPlugin::GPSEventHandler, this);
        gps_nav_solution_subscribers[*i] = nh.subscribe("/"+*i+"/navsol", 10, &RoverGUIPlugin::GPSNavSolutionEventHandler, this);
        rover_diagnostic_subscribers[*i] = nh.subscribe("/"+*i+"/diagnostics", 10, &RoverGUIPlugin::diagnosticEventHandler, this);

        RoverStatus rover_status;
        // Build new ui rover list string
        try
        {
            rover_status = rover_statuses.at(*i);
        }
        catch (std::out_of_range& e)
        {
            emit sendInfoLogMessage("No status entry for rover " + QString::fromStdString(*i));
        }

        QString rover_name_and_status = QString::fromStdString(*i) // Add the rover name
                                                + " (" // Delimiters needed for parsing the rover name and status when read
                                                +  QString::fromStdString(rover_status.status_msg) // Add the rover status
                                                + ")";

        QListWidgetItem* new_item = new QListWidgetItem(rover_name_and_status);
        new_item->setForeground(Qt::green);
        ui.rover_list->addItem(new_item);

        // Create the corresponding diagnostic data listwidgetitem
        QListWidgetItem* new_diags_item = new QListWidgetItem("");

        // The user shouldn't be able to select the diagnostic output
        new_diags_item->setFlags(new_diags_item->flags() & ~Qt::ItemIsSelectable);

        ui.rover_diags_list->addItem(new_diags_item);


        // Add the map selection checkbox for this rover
        QListWidgetItem* new_map_selection_item = new QListWidgetItem("");

        // set checkable but not selectable flags
        new_map_selection_item->setFlags(new_map_selection_item->flags() | Qt::ItemIsUserCheckable);
        new_map_selection_item->setFlags(new_map_selection_item->flags() & ~Qt::ItemIsSelectable);
        new_map_selection_item->setCheckState(Qt::Unchecked);

        // Add to the widget list
        ui.map_selection_list->addItem(new_map_selection_item);

    }
    }

    // If rovers have not sent a status message recently mark them as disconnected
    for(int row = 0; row < ui.rover_list->count(); row++)
    {
        QListWidgetItem *rover_item = ui.rover_list->item(row);
        QListWidgetItem *diags_item = ui.rover_diags_list->item(row);

        // Extract rover name
        string rover_name_and_status = rover_item->text().toStdString();

        // Rover names start at the begining of the rover name and status string and end at the first space
        size_t rover_name_length = rover_name_and_status.find_first_of(" ");
        string ui_rover_name = rover_name_and_status.substr(0, rover_name_length);

        // Check the time of last contact with this rover
        RoverStatus rover_status = rover_statuses[ui_rover_name];
        if (ros::Time::now() - rover_status.timestamp < disconnect_threshold)
        {
            rover_item->setForeground(Qt::green);
        }
        else
        {
            rover_item->setForeground(Qt::red);
	    diags_item->setForeground(Qt::red);

	    diags_item->setText("disconnected");
        }
=======
    for(set<string>::const_iterator i = rover_names.begin(); i != rover_names.end(); ++i)
    {
        QListWidgetItem* new_item = new QListWidgetItem(QString::fromStdString(*i));
        new_item->setForeground(Qt::red);
        ui.rover_list->addItem(new_item);
    }

    setupPublishers();
    setupSubscribers();
}

void RoverGUIPlugin::setupPublishers()
{
    // Set the robot to accept manual control. Latch so even if the robot connects later it will get the message.
    
    if (!selected_rover_name.empty()) {
		string control_mode_topic = "/"+selected_rover_name+"/mode";

		control_mode_publishers[selected_rover_name]=nh.advertise<std_msgs::UInt8>(control_mode_topic, 10, true); // last argument sets latch to true

		string joystick_topic = "/"+selected_rover_name+"/joystick";
		displayLogMessage("Setting up joystick publisher " + QString::fromStdString(joystick_topic));
		joystick_publisher = nh.advertise<geometry_msgs::Twist>(joystick_topic, 10, this);
	}
	    
    set<string>::iterator rover_it;
    for (rover_it = rover_names.begin(); rover_it != rover_names.end(); rover_it++)
    {
		targetPickUpPublisher[*rover_it] = nh.advertise<std_msgs::Int16>("/"+*rover_it+"/targetPickUpValue", 10, this);
		targetDropOffPublisher[*rover_it] = nh.advertise<std_msgs::Int16>("/"+*rover_it+"/targetDropOffValue", 10, this);
	}
}

void RoverGUIPlugin::setupSubscribers()
{
    // Subscriptions for the selected rover
    if (!selected_rover_name.empty())
    {
        // Create a subscriber to listen for camera events
        image_transport::ImageTransport it(nh);
        int frame_rate = 1;
        // Theroa codex results in the least information being transmitted
        camera_subscriber = it.subscribe("/"+selected_rover_name+"/camera/image", frame_rate, &RoverGUIPlugin::cameraEventHandler, this, image_transport::TransportHints("theora"));

        // IMU Subscriptions
        imu_subscriber = nh.subscribe("/"+selected_rover_name+"/imu", 10, &RoverGUIPlugin::IMUEventHandler, this);

        // Ultrasound Subscriptions
        us_center_subscriber = nh.subscribe("/"+selected_rover_name+"/sonarCenter", 10, &RoverGUIPlugin::centerUSEventHandler, this);
        us_left_subscriber = nh.subscribe("/"+selected_rover_name+"/sonarLeft", 10, &RoverGUIPlugin::leftUSEventHandler, this);
        us_right_subscriber = nh.subscribe("/"+selected_rover_name+"/sonarRight", 10, &RoverGUIPlugin::rightUSEventHandler, this);
    }


    // Subscriptions for all rovers

    set<string>::iterator rover_it;
    for (rover_it = rover_names.begin(); rover_it != rover_names.end(); rover_it++)
    {
        obstacle_subscribers[*rover_it] = nh.subscribe("/"+*rover_it+"/obstacle", 10, &RoverGUIPlugin::obstacleEventHandler, this);

        // Odometry and GPS subscribers
        encoder_subscribers[*rover_it] = nh.subscribe("/"+*rover_it+"/odom/", 10, &RoverGUIPlugin::encoderEventHandler, this);
        ekf_subscribers[*rover_it] = nh.subscribe("/"+*rover_it+"/odom/ekf", 10, &RoverGUIPlugin::EKFEventHandler, this);
        gps_subscribers[*rover_it] = nh.subscribe("/"+*rover_it+"/odom/navsat", 10, &RoverGUIPlugin::GPSEventHandler, this);
        // Target subscribers
        targetPickUpSubscribers[*rover_it] = nh.subscribe("/"+*rover_it+"/targetPickUpImage", 10, &RoverGUIPlugin::targetPickUpEventHandler, this);
        targetDropOffSubscribers[*rover_it] = nh.subscribe("/"+*rover_it+"/targetDropOffImage", 10, &RoverGUIPlugin::targetDropOffEventHandler, this);
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
    }
}

void RoverGUIPlugin::centerUSEventHandler(const sensor_msgs::Range::ConstPtr& msg)
{
    // Temp hardcode max and min because setting the max and min on the controller side causes a problem
    float min_range = 0.01;; // meters
    float max_range = 3;


    //ui.us_frame->setCenterRange(msg->range, msg->min_range, msg->max_range);
    ui.us_frame->setCenterRange(msg->range, min_range, max_range);
 }

void RoverGUIPlugin::rightUSEventHandler(const sensor_msgs::Range::ConstPtr& msg)
{
    // Temp hardcode max and min because setting the max and min on the controller side causes a problem
    float min_range = 0.01;
    float max_range = 3;
    //ui.us_frame->setCenterRange(msg->range, min_range, max_range);

    ui.us_frame->setRightRange(msg->range, min_range, max_range);

//    ui.us_frame->setRightRange(msg->range, msg->min_range, msg->max_range);

}

void RoverGUIPlugin::leftUSEventHandler(const sensor_msgs::Range::ConstPtr& msg)
{
    // Temp hardcode max and min because setting the max and min on the controller side causes a problem
    float min_range = 0.01;;
    float max_range = 3;


    //ui.us_frame->setLeftRange(msg->range, msg->min_range, msg->max_range);
    ui.us_frame->setLeftRange(msg->range, min_range, max_range);
}

void RoverGUIPlugin::IMUEventHandler(const sensor_msgs::Imu::ConstPtr& msg)
{
    ui.imu_frame->setLinearAcceleration( msg->linear_acceleration.x,
                                         msg->linear_acceleration.y,
                                         msg->linear_acceleration.z );

    ui.imu_frame->setAngularVelocity(    msg->angular_velocity.x,
                                         msg->angular_velocity.y,
                                         msg->angular_velocity.z    );

    ui.imu_frame->setOrientation(        msg->orientation.w,
                                         msg->orientation.x,
                                         msg->orientation.y,
                                         msg->orientation.z        );

}

<<<<<<< HEAD
// This handler receives data messages from the diagnostics package. It uses a float array to package the
// data for flexibility. This means callers have to know what data is stored at each poistion.
// When the data we cant to display stabalizes we should consider changing this to a custom
// ROS message type that names the data being stored.
// We extract the sender name from the ROS topic name rather than the publisher node name because that
// tends to be more stable. Sometimes teams rename the nodes but renaming the topics would cause
// other problems for them. This is distinct from the diagnostics log handler which received messages rather
// than continual data readings.
void RoverGUIPlugin::diagnosticEventHandler(const ros::MessageEvent<const std_msgs::Float32MultiArray> &event) {

    const std::string& publisher_name = event.getPublisherName();
    const ros::M_string& header = event.getConnectionHeader();
    ros::Time receipt_time = event.getReceiptTime();

    // Extract rover name from the message source
    string topic = header.at("topic");
    size_t found = topic.find("/diagnostics");
    string rover_name = topic.substr(1,found-1);

    const boost::shared_ptr<const std_msgs::Float32MultiArray> msg = event.getMessage();

    string diagnostic_display = "";

    // Read data from the message array
    int wireless_quality = static_cast<int>(msg->data[0]); // Wireless quality is an integer value
    float byte_rate = msg->data[1]; // Bandwidth used by the wireless interface
    float sim_rate = msg->data[2]; // Simulation update rate

    // Declare the output colour variables
    int red = 255;
    int green = 255;
    int blue = 255;
    
    // Check whether there is sim update data. Will be < 0 for the sim rate if a physical rover is sending the data.
    // If so assume the diagnostic data is coming from a simulated rover.
    // TODO: replace with a proper message type so we don't need to use in stream flags like this.
    if ( sim_rate < 0 )
    {
        // Change the color of the text based on the link quality. These numbers are from
        // experience but need tuning. The raw quality value is scaled into a new range to make the colors more meaningful
        int quality_max = 70;
        int quality_min = 0;
        int scaled_max = 10;
        int scaled_min = 0;
        int quality_range = quality_max - quality_min; // Max minus min
        int scaled_range = scaled_max - scaled_min; // Scaled to match the experimental quality of the connection. Below 30 should be red = bad
        int scaled_wireless_quality = (((wireless_quality - quality_min)*static_cast<float>(scaled_range))/quality_range) + scaled_min; // scale the quality to the new range

        if (scaled_range != 0)
        {
            green = 255 * scaled_wireless_quality/static_cast<float>(scaled_range);
            red = 255 * (2*scaled_range - (scaled_wireless_quality))/static_cast<float>(2*scaled_range);
        }
        else
        {
            green = 0;
            red = 0;
        }

        blue = 0;

        // Make sure color is in a valid range
        if (green > 255) green = 255;
        if (blue > 255) blue = 255;
        if (red > 255) red = 255;

        if (green < 0) green = 0;
        if (blue < 0) blue = 0;
        if (red < 0) red = 0;

        diagnostic_display = to_string(wireless_quality);

        // Convert the byte rate into a string with units
        // Rate in B/s
        float rate = byte_rate;

        // Conversion factors to make the rate human friendly
        int KB = 1024;
        int MB = 1024*1024;

        string rate_str;
        string units;
        if (rate < KB) {
            rate_str = to_string(rate);
            units = "B/s";
        } else if (rate < MB) {
            rate = rate/KB;
            units = "KB/s";
        } else {
            rate = rate/MB;
            units = "MB/s";
        }

        rate_str = to_string(rate);

        if (rate_str[rate_str.find(".")+1] != '0')
            rate_str = rate_str.erase(rate_str.find(".")+2,string::npos);
        else
            rate_str = rate_str.erase(rate_str.find("."),string::npos);

        diagnostic_display += " | " + rate_str + " " + units;

    }
    else
    {
        string sim_rate_str = to_string(sim_rate);

        // Truncate to 1 digit
        if (sim_rate_str[sim_rate_str.find(".")+1] != '0')
            sim_rate_str = sim_rate_str.erase(sim_rate_str.find(".")+3,string::npos);
        else
            sim_rate_str = sim_rate_str.erase(sim_rate_str.find("."),string::npos);

        red = 255*(1-sim_rate);
        green = 255*sim_rate;
        blue = 0;

        // Make sure color is in a valid range
        if (green > 255) green = 255;
        if (blue > 255) blue = 255;
        if (red > 255) red = 255;

        if (green < 0) green = 0;
        if (blue < 0) blue = 0;
        if (red < 0) red = 0;

        diagnostic_display = sim_rate_str + " sim rate";
    }

    emit sendDiagsDataUpdate(QString::fromStdString(rover_name), QString::fromStdString(diagnostic_display), QColor(red, green, blue));

}

// We use item changed signal as a proxy for the checkbox being clicked
void RoverGUIPlugin::mapSelectionListItemChangedHandler(QListWidgetItem* changed_item)
{
    // Get the rover name associated with this map selction list item
    int row = ui.map_selection_list->row(changed_item);

    // Rover names start at the begining of the rover name and status string and end at the first space
    QListWidgetItem* rover_item = ui.rover_list->item(row);

    // Extract the rover name corresponding to the changed map selection item
    string rover_name_and_status = rover_item->text().toStdString();

    // Rover names start at the begining of the rover name and status string and end at the first space
    size_t rover_name_length = rover_name_and_status.find_first_of(" ");
    string ui_rover_name = rover_name_and_status.substr(0, rover_name_length);

    bool checked = changed_item->checkState();

    emit sendInfoLogMessage("Map selection changed to " + (checked ? QString("true") : QString("false")) + " for rover " + QString::fromStdString(ui_rover_name));

    ui.map_frame->setWhetherToDisplay(ui_rover_name, checked);
}

=======
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
void RoverGUIPlugin::GPSCheckboxToggledEventHandler(bool checked)
{
    ui.map_frame->setDisplayGPSData(checked);
}

void RoverGUIPlugin::EKFCheckboxToggledEventHandler(bool checked)
{
    ui.map_frame->setDisplayEKFData(checked);
}

void RoverGUIPlugin::encoderCheckboxToggledEventHandler(bool checked)
{
    ui.map_frame->setDisplayEncoderData(checked);
}

<<<<<<< HEAD
void RoverGUIPlugin::displayDiagLogMessage(QString msg)
{
    if (msg.isEmpty()) msg = "Message is empty";
    if (msg == NULL) msg = "Message was a NULL pointer";


     // replace new lines with <BR> in the message. Uppercase in order to differentiate from <br> below
    msg.replace("\n","<BR>");


    // Prevent the log from growning too large. Maintain a maximum specified size
    // by removing characters from the beginning of the log.
    // Calculate the number of characters in the log. If the log size is larger than the max size specified
    // then find the position of the first newline that reduces the log size to less than the max size.
    // Delete all characters up to that position.
    int overflow = diag_log_messages.size() - max_diag_log_length; 
    
    // Get the position of the the first newline after the overflow amount
    int newline_pos = diag_log_messages.indexOf( "<br>", overflow, Qt::CaseSensitive );
    
    // If the max size is exceeded and the number of characters to remove is less than
    // the size of the log remove those characters.
    if ( overflow > 0 && newline_pos < diag_log_messages.size() ) {   
      diag_log_messages.remove(0, newline_pos);
    }

    diag_log_messages += "<font color='white'>"+msg+"</font><br>";

    ui.diag_log->setText(diag_log_messages);

    QScrollBar *sb = ui.diag_log->verticalScrollBar();
    sb->setValue(sb->maximum());
}

void RoverGUIPlugin::displayInfoLogMessage(QString msg)
=======
// Currently broken. Calling displayLogMessage from the ROS event thread causes a crash or hang
//void RoverGUIPlugin::targetDetectedEventHandler(rover_onboard_target_detection::ATag tagInfo) //rover_onboard_target_detection::ATag msg )
//{
//    // Just let the user know the event happened
//   // displayLogMessage("Tag detected");

//}

void RoverGUIPlugin::displayLogMessage(QString msg)
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
{
    if (msg.isEmpty()) msg = "Message is empty";
    if (msg == NULL) msg = "Message was a NULL pointer";

<<<<<<< HEAD
    // replace new lines with <BR> in the message. Uppercase in order to differentiate from <br> below
    msg.replace("\n","<BR>");

    // Prevent the log from growning too large. Maintain a maximum specified size
    // by removing characters from the beginning of the log.
    // Calculate the number of characters in the log. If the log size is larger than the max size specified
    // then find the position of the first newline that reduces the log size to less than the max size.
    // Delete all characters up to that position.
    int overflow = info_log_messages.size() - max_info_log_length; 
    
    // Get the position of the the first newline after the overflow amount
    int newline_pos = info_log_messages.indexOf( "<br>", overflow, Qt::CaseSensitive );
    
    // If the max size is exceeded and the number of characters to remove is less than
    // the size of the log remove those characters.
    if ( overflow > 0 && newline_pos < info_log_messages.size() ) {   
      info_log_messages.remove(0, newline_pos);
    }
    
    // Use the <br> tag to make log messages atomic for easier deletion later.
    info_log_messages += "<font color='white'>"+msg+"</font><br>";

    ui.info_log->setText(info_log_messages);

    QScrollBar *sb = ui.info_log->verticalScrollBar();
    sb->setValue(sb->maximum());
}

// These button handlers allow the user to select whether to manually pan and zoom the map
// or use auto scaling.
void RoverGUIPlugin::mapAutoRadioButtonEventHandler(bool marked)
{
    if (!marked) return;
    ui.map_frame->setAutoTransform();

}

void RoverGUIPlugin::mapManualRadioButtonEventHandler(bool marked)
{
    if (!marked) return;
    ui.map_frame->setManualTransform();
}


=======

    // replace new lines with <br> in the message
    msg.replace("\n","<br>");

    QString new_message = msg+"<br>";
    log_messages = log_messages+new_message;
    ui.log->setText("<font color='white'>"+log_messages+"</font>");

    QScrollBar *sb = ui.log->verticalScrollBar();
    sb->setValue(sb->maximum());
}

>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
void RoverGUIPlugin::autonomousRadioButtonEventHandler(bool marked)
{
    if (!marked) return;

    rover_control_state[selected_rover_name] = 2;
<<<<<<< HEAD
=======
    setupPublishers();
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea

    std_msgs::UInt8 control_mode_msg;
    control_mode_msg.data = 2; // 2 indicates autonomous control

    control_mode_publishers[selected_rover_name].publish(control_mode_msg);
<<<<<<< HEAD
    emit sendInfoLogMessage(QString::fromStdString(selected_rover_name)+" changed to autonomous control");

    QString return_msg = stopROSJoyNode();
    emit sendInfoLogMessage(return_msg);
=======
    displayLogMessage(QString::fromStdString(selected_rover_name)+" changed to autonomous control");

    QString return_msg = stopROSJoyNode();
    displayLogMessage(return_msg);
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
    
    //Enable all stop button
    ui.all_stop_button->setEnabled(true);
    ui.all_stop_button->setStyleSheet("color: white; border:2px solid white;");
<<<<<<< HEAD

    //Hide joystick frame
    ui.joystick_frame->setHidden(true);
=======
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
}

void RoverGUIPlugin::joystickRadioButtonEventHandler(bool marked)
{
    if (!marked) return;

    rover_control_state[selected_rover_name] = 1;
<<<<<<< HEAD
    emit sendInfoLogMessage("Setting up joystick publisher " + QString::fromStdString("/"+selected_rover_name+"/joystick"));

    // Setup joystick publisher
    joystick_publisher = nh.advertise<sensor_msgs::Joy>("/"+selected_rover_name+"/joystick", 10, this);

    // Setup Gripper publishers

    // Have to allocate the joystickGripperInterface on the heap because it derives from QObject which disallows copy constructors
    // Lock this section to prevent the inferface from being changed while in use

    if (joystickGripperInterface != NULL)
    {
        emit sendInfoLogMessage("Redirecting existing Joystick Gripper Interface to " + QString::fromStdString(selected_rover_name));
        joystickGripperInterface->changeRovers(selected_rover_name);
    }
    else
    {
        emit sendInfoLogMessage("Setting up Joystick Gripper Interface for  " + QString::fromStdString(selected_rover_name));
        joystickGripperInterface = new JoystickGripperInterface(nh, selected_rover_name);
    }
=======
    setupPublishers();
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea

    std_msgs::UInt8 control_mode_msg;
    control_mode_msg.data = 1; // 1 indicates manual control

    control_mode_publishers[selected_rover_name].publish(control_mode_msg);
<<<<<<< HEAD
    emit sendInfoLogMessage(QString::fromStdString(selected_rover_name)+" changed to joystick control");\

    QString return_msg = startROSJoyNode();
    emit sendInfoLogMessage(return_msg);
=======
    displayLogMessage(QString::fromStdString(selected_rover_name)+" changed to joystick control");\

    QString return_msg = startROSJoyNode();
    displayLogMessage(return_msg);
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
    
    //Enable all autonomous button
    ui.all_autonomous_button->setEnabled(true);
    ui.all_autonomous_button->setStyleSheet("color: white; border:2px solid white;");
<<<<<<< HEAD
    
    //Show joystick frame
    ui.joystick_frame->setHidden(false);
=======
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
}

void RoverGUIPlugin::allAutonomousButtonEventHandler()
{
<<<<<<< HEAD
    emit sendInfoLogMessage("changing all rovers to autonomous control...");
=======
    displayLogMessage("changing all rovers to autonomous control...");
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea

    std::string remember_selected_rover_name = selected_rover_name;
    int remember_selected_index = ui.rover_list->currentRow();
    int selected_index = -1; // zero array indexing, ensure last selected index is in range

    // manually trigger the autonomous radio button event for all rovers
    for (set<string>::iterator it = rover_names.begin(); it != rover_names.end(); it++)
    {
        selected_index++;
        selected_rover_name = *it;
        autonomousRadioButtonEventHandler(true);
    }

    // trigger the current rover changed event:
    // if we previously selected a rover, keep that rover selected
    if (remember_selected_index >= 0)
    {
        selected_rover_name = remember_selected_rover_name;
        ui.rover_list->setCurrentItem(ui.rover_list->item(remember_selected_index));
    }
    // otherwise, default to the last rover in the rover list
    else
    {
        ui.rover_list->setCurrentItem(ui.rover_list->item(selected_index));
    }

    ui.joystick_control_radio_button->setEnabled(true);
    ui.autonomous_control_radio_button->setEnabled(true);
    ui.joystick_control_radio_button->setChecked(false);
    ui.autonomous_control_radio_button->setChecked(true);
    ui.joystick_frame->setHidden(true);
    
    //Disable all autonomous button
    ui.all_autonomous_button->setEnabled(false);
    ui.all_autonomous_button->setStyleSheet("color: grey; border:2px solid grey;");
<<<<<<< HEAD

    //Experiment Timer START

    // this catches the case when the /clock timer is not running
    // AKA: when we are not running a simulation
    if (current_simulated_time_in_seconds > 0.0) {
        if (ui.simulation_timer_combo_box->currentText() == "no time limit") {
            timer_start_time_in_seconds = 0.0;
            timer_stop_time_in_seconds = 0.0;
            is_timer_on = false;
        } else if (ui.simulation_timer_combo_box->currentText() == "10 min (Testing)") {
            timer_start_time_in_seconds = current_simulated_time_in_seconds;
            timer_stop_time_in_seconds = timer_start_time_in_seconds + 600.0;
            is_timer_on = true;
            emit sendInfoLogMessage("\nSetting experiment timer to start at: " +
                                    QString::number(getHours(timer_start_time_in_seconds)) + " hours, " +
                                    QString::number(getMinutes(timer_start_time_in_seconds)) + " minutes, " +
                                    QString::number(getSeconds(timer_start_time_in_seconds)) + " seconds");
            ui.simulationTimerStartLabel->setText("<font color='white'>" +
                                                  QString::number(getHours(timer_start_time_in_seconds)) + " hours, " +
                                                  QString::number(getMinutes(timer_start_time_in_seconds)) + " minutes, " +
                                                  QString::number(floor(getSeconds(timer_start_time_in_seconds))) + " seconds</font>");
            emit sendInfoLogMessage("Setting experiment timer to stop at: " +
                                    QString::number(getHours(timer_stop_time_in_seconds)) + " hours, " +
                                    QString::number(getMinutes(timer_stop_time_in_seconds)) + " minutes, " +
                                    QString::number(getSeconds(timer_stop_time_in_seconds)) + " seconds\n");
            ui.simulationTimerStopLabel->setText("<font color='white'>" +
                                                 QString::number(getHours(timer_stop_time_in_seconds)) + " hours, " +
                                                 QString::number(getMinutes(timer_stop_time_in_seconds)) + " minutes, " +
                                                 QString::number(floor(getSeconds(timer_stop_time_in_seconds))) + " seconds</font>");
            ui.simulation_timer_combo_box->setEnabled(false);
            ui.simulation_timer_combo_box->setStyleSheet("color: grey; border:2px solid grey;");
        } else if (ui.simulation_timer_combo_box->currentText() == "30 min (Preliminary)") {
            timer_start_time_in_seconds = current_simulated_time_in_seconds;
            timer_stop_time_in_seconds = timer_start_time_in_seconds + 1800.0;
            is_timer_on = true;
            emit sendInfoLogMessage("\nSetting experiment timer to start at: " +
                                    QString::number(getHours(timer_start_time_in_seconds)) + " hours, " +
                                    QString::number(getMinutes(timer_start_time_in_seconds)) + " minutes, " +
                                    QString::number(getSeconds(timer_start_time_in_seconds)) + " seconds");
            ui.simulationTimerStartLabel->setText("<font color='white'>" +
                                                  QString::number(getHours(timer_start_time_in_seconds)) + " hours, " +
                                                  QString::number(getMinutes(timer_start_time_in_seconds)) + " minutes, " +
                                                  QString::number(floor(getSeconds(timer_start_time_in_seconds))) + " seconds</font>");
            emit sendInfoLogMessage("Setting experiment timer to stop at: " +
                                    QString::number(getHours(timer_stop_time_in_seconds)) + " hours, " +
                                    QString::number(getMinutes(timer_stop_time_in_seconds)) + " minutes, " +
                                    QString::number(getSeconds(timer_stop_time_in_seconds)) + " seconds\n");
            ui.simulationTimerStopLabel->setText("<font color='white'>" +
                                                 QString::number(getHours(timer_stop_time_in_seconds)) + " hours, " +
                                                 QString::number(getMinutes(timer_stop_time_in_seconds)) + " minutes, " +
                                                 QString::number(floor(getSeconds(timer_stop_time_in_seconds))) + " seconds</font>");
            ui.simulation_timer_combo_box->setEnabled(false);
            ui.simulation_timer_combo_box->setStyleSheet("color: grey; border:2px solid grey;");
        } else if (ui.simulation_timer_combo_box->currentText() == "60 min (Final)") {
            timer_start_time_in_seconds = current_simulated_time_in_seconds;
            timer_stop_time_in_seconds = timer_start_time_in_seconds + 3600.0;
            is_timer_on = true;
            emit sendInfoLogMessage("\nSetting experiment timer to start at: " +
                                    QString::number(getHours(timer_start_time_in_seconds)) + " hours, " +
                                    QString::number(getMinutes(timer_start_time_in_seconds)) + " minutes, " +
                                    QString::number(getSeconds(timer_start_time_in_seconds)) + " seconds");
            ui.simulationTimerStartLabel->setText("<font color='white'>" +
                                                  QString::number(getHours(timer_start_time_in_seconds)) + " hours, " +
                                                  QString::number(getMinutes(timer_start_time_in_seconds)) + " minutes, " +
                                                  QString::number(floor(getSeconds(timer_start_time_in_seconds))) + " seconds</font>");
            emit sendInfoLogMessage("Setting experiment timer to stop at: " +
                                    QString::number(getHours(timer_stop_time_in_seconds)) + " hours, " +
                                    QString::number(getMinutes(timer_stop_time_in_seconds)) + " minutes, " +
                                    QString::number(getSeconds(timer_stop_time_in_seconds)) + " seconds\n");
            ui.simulationTimerStopLabel->setText("<font color='white'>" +
                                                 QString::number(getHours(timer_stop_time_in_seconds)) + " hours, " +
                                                 QString::number(getMinutes(timer_stop_time_in_seconds)) + " minutes, " +
                                                 QString::number(floor(getSeconds(timer_stop_time_in_seconds))) + " seconds</font>");
            ui.simulation_timer_combo_box->setEnabled(false);
            ui.simulation_timer_combo_box->setStyleSheet("color: grey; border:2px solid grey;");
        }
    }
    //Experiment Timer END
}

double RoverGUIPlugin::getHours(double seconds) {
    if (seconds < 3600.0) return 0.0;

    double hours = 0.0;

    while (seconds >= 3600.0) {
        seconds -= 3600.0;
        hours++;
    }

    return hours;
}

double RoverGUIPlugin::getMinutes(double seconds) {
    double hours = getHours(seconds);
    seconds -= hours * 3600.0;

    if (seconds < 60.0) return 0.0;

    double minutes = 0.0;

    while (seconds >= 60.0) {
        seconds -= 60.0;
        minutes++;
    }

    return minutes;
}

double RoverGUIPlugin::getSeconds(double seconds) {
    double hours = getHours(seconds);
    double minutes = getMinutes(seconds);
    seconds -= (hours * 3600.0) + (minutes * 60.0);

    return seconds;
=======
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
}

void RoverGUIPlugin::allStopButtonEventHandler()
{
<<<<<<< HEAD
    emit sendInfoLogMessage("changing all rovers to manual control...");
=======
    displayLogMessage("changing all rovers to manual control...");
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea

    std::string remember_selected_rover_name = selected_rover_name;
    int remember_selected_index = ui.rover_list->currentRow();
    int selected_index = -1; // zero array indexing, ensure last selected index is in range

    // manually trigger the manual radio button event for all rovers
    for (set<string>::iterator it = rover_names.begin(); it != rover_names.end(); it++)
    {
        selected_index++;
        selected_rover_name = *it;
        joystickRadioButtonEventHandler(true);
    }

    // trigger the current rover changed event:
    // if we previously selected a rover, keep that rover selected
    if (remember_selected_index >= 0)
    {
        selected_rover_name = remember_selected_rover_name;
        ui.rover_list->setCurrentItem(ui.rover_list->item(remember_selected_index));
    }
    // otherwise, default to the last rover in the rover list
    else
    {
        ui.rover_list->setCurrentItem(ui.rover_list->item(selected_index));
    }

    ui.joystick_control_radio_button->setEnabled(true);
    ui.autonomous_control_radio_button->setEnabled(true);
    ui.joystick_control_radio_button->setChecked(true);
    ui.autonomous_control_radio_button->setChecked(false);
    ui.joystick_frame->setHidden(false);
    
    //Disable all stop button
    ui.all_stop_button->setEnabled(false);
    ui.all_stop_button->setStyleSheet("color: grey; border:2px solid grey;");
<<<<<<< HEAD

    // reset the simulation timer variables
    ui.simulation_timer_combo_box->setEnabled(true);
    ui.simulation_timer_combo_box->setStyleSheet("color: white; border:1px solid white; padding: 1px 0px 1px 3px");
    ui.simulationTimerStartLabel->setText("<font color='white'>---</font>");
    ui.simulationTimerStopLabel->setText("<font color='white'>---</font>");
    timer_start_time_in_seconds = 0.0;
    timer_stop_time_in_seconds = 0.0;

    if (is_timer_on == true) {
        is_timer_on = false;
        emit sendInfoLogMessage("\nSimulation timer cancelled at: " +
                                QString::number(getHours(current_simulated_time_in_seconds)) + " hours, " +
                                QString::number(getMinutes(current_simulated_time_in_seconds)) + " minutes, " +
                                QString::number(getSeconds(current_simulated_time_in_seconds)) + " seconds\n");
    }
}

// Get the path to the world file containing the custom distribution from the user
void RoverGUIPlugin::customWorldButtonEventHandler()
{
    const char *name = "SWARMATHON_APP_ROOT";
    char *app_root_cstr;
    app_root_cstr = getenv(name);
    QString app_root = QString(app_root_cstr) + "/simulation/worlds/";

    QString path = QFileDialog::getOpenFileName(widget, tr("Open File"),
                                                    app_root,
                                                    tr("Gazebo World File (*.world)"));

    sim_mgr.setCustomWorldPath(path);
    emit sendInfoLogMessage("User selected custom world path: " + path);

    // Extract the base filename for short display
    QFileInfo fi=path;
    ui.custom_world_path->setText(fi.baseName());
}

// Enable or disable custom distributions
void RoverGUIPlugin::customWorldRadioButtonEventHandler(bool toggled)
{
    ui.custom_world_path_button->setEnabled(toggled);

    // Set the button color to reflect whether or not it is disabled
    // Clear the sim path if custom distribution it deselected
    if( toggled )
    {
        ui.custom_world_path_button->setStyleSheet("color: white; border:2px solid white;");
    }
    else
    {
        sim_mgr.setCustomWorldPath("");
        ui.custom_world_path->setText("");
        ui.custom_world_path_button->setStyleSheet("color: grey; border:2px solid grey;");
    }
}

void RoverGUIPlugin::mapPopoutButtonEventHandler()
{
    ui.map_frame->popout();
=======
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
}

void RoverGUIPlugin::buildSimulationButtonEventHandler()
{
<<<<<<< HEAD
    emit sendInfoLogMessage("Building simulation...");

    ui.build_simulation_button->setEnabled(false);
=======
    displayLogMessage("Building simulation...");

    ui.build_simulation_button->setEnabled(false);

>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
    ui.build_simulation_button->setStyleSheet("color: grey; border:2px solid grey;");

    QString return_msg;

    if (sim_mgr.isGazeboServerRunning())
    {
<<<<<<< HEAD
        emit sendInfoLogMessage("A gazebo server simulation process is already running. Restart the Swarmathon GUI to clear.");
        return;
    }

=======
        displayLogMessage("A gazebo server simulation process is already running. Restart the Swarmathon GUI to clear.");
        return;
    }

    // Initialize the target counts
    ui.num_targets_collected_label->setText(QString("<font color='white'>0</font>"));
    ui.num_targets_detected_label->setText(QString("<font color='white'>0</font>"));
    targetsPickedUp.clear();
    targetsDroppedOff.clear();

>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
    QProcess* sim_server_process = sim_mgr.startGazeboServer();
    connect(sim_server_process, SIGNAL(finished(int)), this, SLOT(gazeboServerFinishedEventHandler()));


    if (ui.final_radio_button->isChecked())
    {
         arena_dim = 23.1;
         addFinalsWalls();
    }
    else
    {
        arena_dim = 15;
        addPrelimsWalls();
    }

<<<<<<< HEAD
    emit sendInfoLogMessage(QString("Set arena size to ")+QString::number(arena_dim)+"x"+QString::number(arena_dim));

    if (ui.texture_combobox->currentText() == "Gravel")
    {
    emit sendInfoLogMessage("Adding gravel ground plane...");
    return_msg = sim_mgr.addGroundPlane("mars_ground_plane");
    emit sendInfoLogMessage(return_msg);
    }
    else if (ui.texture_combobox->currentText() == "KSC Concrete")
    {
    emit sendInfoLogMessage("Adding concrete ground plane...");
    return_msg = sim_mgr.addGroundPlane("concrete_ground_plane");
    emit sendInfoLogMessage(return_msg);
    }
    else if (ui.texture_combobox->currentText() == "Car park")
    {
    emit sendInfoLogMessage("Adding carpark ground plane...");
    return_msg = sim_mgr.addGroundPlane("carpark_ground_plane");
    emit sendInfoLogMessage(return_msg);
    }
    else
    {
        emit sendInfoLogMessage("Unknown ground plane...");
    }


    emit sendInfoLogMessage("Adding collection disk...");
    float collection_disk_radius = 0.5; // meters
    sim_mgr.addModel("collection_disk", "collection_disk", 0, 0, 0, collection_disk_radius);
    score_subscriber = nh.subscribe("/collectionZone/score", 10, &RoverGUIPlugin::scoreEventHandler, this);
    simulation_timer_subscriber = nh.subscribe("/clock", 10, &RoverGUIPlugin::simulationTimerEventHandler, this);
=======
    displayLogMessage(QString("Set arena size to ")+QString::number(arena_dim)+"x"+QString::number(arena_dim));

    if (ui.texture_combobox->currentText() == "Gravel")
    {
    displayLogMessage("Adding gravel ground plane...");
    return_msg = sim_mgr.addGroundPlane("mars_ground_plane");
    displayLogMessage(return_msg);
    }
    else if (ui.texture_combobox->currentText() == "KSC Concrete")
    {
    displayLogMessage("Adding concrete ground plane...");
    return_msg = sim_mgr.addGroundPlane("concrete_ground_plane");
    displayLogMessage(return_msg);
    }
    else if (ui.texture_combobox->currentText() == "Car park")
    {
    displayLogMessage("Adding carpark ground plane...");
    return_msg = sim_mgr.addGroundPlane("carpark_ground_plane");
    displayLogMessage(return_msg);
    }
    else
    {
        displayLogMessage("Unknown ground plane...");
    }


    displayLogMessage("Adding collection disk...");
    float collection_disk_radius = 0.5; // meters
    sim_mgr.addModel("collection_disk", "collection_disk", 0, 0, 0, collection_disk_radius);
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea

    int n_rovers_created = 0;
    int n_rovers = 3;
    if (ui.final_radio_button->isChecked()) n_rovers = 6;

<<<<<<< HEAD
    // If the user chose to override the number of rovers to add to the simulation read the selected value
    if (ui.override_num_rovers_checkbox->isChecked()) n_rovers = ui.custom_num_rovers_combobox->currentText().toInt();

=======
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
    QProgressDialog progress_dialog;
    progress_dialog.setWindowTitle("Creating rovers");
    progress_dialog.setCancelButton(NULL); // no cancel button
    progress_dialog.setWindowModality(Qt::ApplicationModal);
    progress_dialog.setWindowFlags(progress_dialog.windowFlags() | Qt::WindowStaysOnTopHint);
    progress_dialog.resize(500, 50);
    progress_dialog.show();

<<<<<<< HEAD
    QString rovers[6] = {"achilles", "aeneas", "ajax", "diomedes", "hector", "paris"};
    QPointF rover_positions[6] = {QPointF(0.0,1.0), QPointF(1.0,1.0), QPointF(1.0,0.0), QPointF(-1.0,0.0), QPointF(0.0,-1.0), QPointF(-1.0,-1.0)};

    // Add rovers to the simulation and start the associated ROS nodes
    for (int i = 0; i < n_rovers; i++)
    {
        emit sendInfoLogMessage("Adding rover "+rovers[i]+"...");
        return_msg = sim_mgr.addRover(rovers[i], rover_positions[i].x(), rover_positions[i].y(), 0);
        emit sendInfoLogMessage(return_msg);

        emit sendInfoLogMessage("Starting rover node for "+rovers[i]+"...");
        return_msg = sim_mgr.startRoverNode(rovers[i]);
        emit sendInfoLogMessage(return_msg);

        progress_dialog.setValue((++n_rovers_created)*100.0f/n_rovers);
        qApp->processEvents(QEventLoop::ExcludeUserInputEvents);
    }

   if (ui.powerlaw_distribution_radio_button->isChecked())
   {
       emit sendInfoLogMessage("Adding powerlaw distribution of targets...");
       return_msg = addPowerLawTargets();
       emit sendInfoLogMessage(return_msg);
   }
   else if (ui.uniform_distribution_radio_button->isChecked())
   {
       emit sendInfoLogMessage("Adding uniform distribution of targets...");
       return_msg = addUniformTargets();
       emit sendInfoLogMessage(return_msg);
   }
   else if (ui.clustered_distribution_radio_button->isChecked())
   {
       emit sendInfoLogMessage("Adding clustered distribution of targets...");
       return_msg = addClusteredTargets();
       emit sendInfoLogMessage(return_msg);
=======
    displayLogMessage("Adding rover achilles...");
    return_msg = sim_mgr.addRover("achilles", 0, 1, 0);
    displayLogMessage(return_msg);

    displayLogMessage("Starting rover node for achilles...");
    return_msg = sim_mgr.startRoverNode("achilles");
    displayLogMessage(return_msg);

    progress_dialog.setValue((++n_rovers_created)*100.0f/n_rovers);
    qApp->processEvents(QEventLoop::ExcludeUserInputEvents);

    displayLogMessage("Adding rover aeneas...");
    return_msg = sim_mgr.addRover("aeneas", -1, 0, 0);
    displayLogMessage(return_msg);

    displayLogMessage("Starting rover node for aeneas...");
    return_msg = sim_mgr.startRoverNode("aeneas");
    displayLogMessage(return_msg);

    progress_dialog.setValue((++n_rovers_created)*100.0f/n_rovers);
    qApp->processEvents(QEventLoop::ExcludeUserInputEvents);

    displayLogMessage("Adding rover ajax...");
    return_msg = sim_mgr.addRover("ajax", 1, 0, 0);
    displayLogMessage(return_msg);

   displayLogMessage("Starting rover node for ajax...");
   return_msg = sim_mgr.startRoverNode("ajax");
   displayLogMessage(return_msg);

   progress_dialog.setValue((++n_rovers_created)*100.0f/n_rovers);
   qApp->processEvents(QEventLoop::ExcludeUserInputEvents);

   if (ui.final_radio_button->isChecked())
   {

       displayLogMessage("Adding rover diomedes...");
       return_msg = sim_mgr.addRover("diomedes", 1, 1, 0);
       displayLogMessage(return_msg);

       displayLogMessage("Starting rover node for diomedes...");
       return_msg = sim_mgr.startRoverNode("diomedes");
       displayLogMessage(return_msg);

       progress_dialog.setValue((++n_rovers_created)*100.0f/n_rovers);
       qApp->processEvents(QEventLoop::ExcludeUserInputEvents);

       displayLogMessage("Adding rover hector...");
       return_msg = sim_mgr.addRover("hector", -1, -1, 0);
       displayLogMessage(return_msg);

       displayLogMessage("Starting rover node for hector...");
       return_msg = sim_mgr.startRoverNode("hector");
       displayLogMessage(return_msg);

        progress_dialog.setValue((++n_rovers_created)*100.0f/n_rovers);
        qApp->processEvents(QEventLoop::ExcludeUserInputEvents);

       displayLogMessage("Adding rover paris...");
       return_msg = sim_mgr.addRover("paris", 1, -1, 0);
       displayLogMessage(return_msg);

       displayLogMessage("Starting rover node for paris...");
       return_msg = sim_mgr.startRoverNode("paris");
       displayLogMessage(return_msg);

        progress_dialog.setValue((++n_rovers_created)*100.0f/n_rovers);
        qApp->processEvents(QEventLoop::ExcludeUserInputEvents);

}
   if (ui.powerlaw_distribution_radio_button->isChecked())
   {
       displayLogMessage("Adding powerlaw distribution of targets...");
       return_msg = addPowerLawTargets();
       displayLogMessage(return_msg);
   }
   else if (ui.uniform_distribution_radio_button->isChecked())
   {
       displayLogMessage("Adding uniform distribution of targets...");
       return_msg = addUniformTargets();
       displayLogMessage(return_msg);
   }
   else if (ui.clustered_distribution_radio_button->isChecked())
   {
       displayLogMessage("Adding clustered distribution of targets...");
       return_msg = addClusteredTargets();
       displayLogMessage(return_msg);
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
   }

   // add walls given nw corner (x,y) and height and width (in meters)

   //addWalls(-arena_dim/2, -arena_dim/2, arena_dim, arena_dim);

   //   // Test rover movement
<<<<<<< HEAD
   //   displayLogMessage("Moving aeneas");
   //   return_msg = sim_mgr.moveRover("aeneas", 10, 0, 0);
   //   displayLogMessage(return_msg);
=======
//   displayLogMessage("Moving aeneas");
//   return_msg = sim_mgr.moveRover("aeneas", 10, 0, 0);
//   displayLogMessage(return_msg);
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea

   //displayLogMessage("Starting the gazebo client to visualize the simulation.");
   //sim_mgr.startGazeboClient();

   ui.visualize_simulation_button->setEnabled(true);
   ui.clear_simulation_button->setEnabled(true);

   ui.visualize_simulation_button->setStyleSheet("color: white;border:1px solid white;");
   ui.clear_simulation_button->setStyleSheet("color: white;border:1px solid white;");

<<<<<<< HEAD
    ui.simulation_timer_combo_box->setEnabled(true);
    ui.simulation_timer_combo_box->setStyleSheet("color: white; border:1px solid white; padding: 1px 0px 1px 3px");

   emit sendInfoLogMessage("Finished building simulation.");

   // Visualize the simulation by default call button event handler
   visualizeSimulationButtonEventHandler();
=======
   displayLogMessage("Finished building simulation.");

  // Visualize the simulation by default call button event handler
   visualizeSimulationButtonEventHandler();

>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
}

void RoverGUIPlugin::clearSimulationButtonEventHandler()
{
    if (!sim_mgr.isGazeboServerRunning())
    {
<<<<<<< HEAD
        emit sendInfoLogMessage("Simulation is not running.");
=======
        displayLogMessage("Simulation is not running.");
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea

        return;
    }

<<<<<<< HEAD
    emit sendInfoLogMessage("Ending simulation...");
=======
    displayLogMessage("Ending simulation...");
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea

    QProgressDialog progress_dialog;
    progress_dialog.setWindowTitle("Shutting Down Rovers");
    progress_dialog.setCancelButton(NULL); // no cancel button
    progress_dialog.setWindowModality(Qt::ApplicationModal);
    progress_dialog.setWindowFlags(progress_dialog.windowFlags() | Qt::WindowStaysOnTopHint);
    progress_dialog.resize(500, 50);
    progress_dialog.show();

    QString return_msg;
    float count = 0.0f;

    // Make a copy of the rover names because stopRoverNode will cause the original set to change
    set<string> rover_names_copy = rover_names;

    for(set<string>::const_iterator i = rover_names_copy.begin(); i != rover_names_copy.end(); ++i)
    {
        return_msg += sim_mgr.stopRoverNode(QString::fromStdString(*i));
        return_msg += "<br>";
        progress_dialog.setValue((++count)*100.0f/rover_names_copy.size());
        qApp->processEvents(QEventLoop::ExcludeUserInputEvents);
    }

    // Unsubscribe from topics

<<<<<<< HEAD
    emit sendInfoLogMessage("Shutting down subscribers...");

    for (map<string,ros::Subscriber>::iterator it=encoder_subscribers.begin(); it!=encoder_subscribers.end(); ++it) 
      {
	qApp->processEvents(QEventLoop::ExcludeUserInputEvents);
	it->second.shutdown();
      }

    encoder_subscribers.clear();

    for (map<string,ros::Subscriber>::iterator it=gps_subscribers.begin(); it!=gps_subscribers.end(); ++it) {
      qApp->processEvents(QEventLoop::ExcludeUserInputEvents);
      it->second.shutdown();
    }

    for (map<string,ros::Subscriber>::iterator it=gps_nav_solution_subscribers.begin(); it!=gps_nav_solution_subscribers.end(); ++it) {
      qApp->processEvents(QEventLoop::ExcludeUserInputEvents);
      it->second.shutdown();
    }

    gps_subscribers.clear();
    gps_nav_solution_subscribers.clear();

    for (map<string,ros::Subscriber>::iterator it=ekf_subscribers.begin(); it!=ekf_subscribers.end(); ++it) {
      qApp->processEvents(QEventLoop::ExcludeUserInputEvents);
      it->second.shutdown();
    }

=======
    displayLogMessage("Shutting down subscribers...");

    for (map<string,ros::Subscriber>::iterator it=encoder_subscribers.begin(); it!=encoder_subscribers.end(); ++it) it->second.shutdown();
    encoder_subscribers.clear();
    for (map<string,ros::Subscriber>::iterator it=gps_subscribers.begin(); it!=gps_subscribers.end(); ++it) it->second.shutdown();
    gps_subscribers.clear();
    for (map<string,ros::Subscriber>::iterator it=ekf_subscribers.begin(); it!=ekf_subscribers.end(); ++it) it->second.shutdown();
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
    ekf_subscribers.clear();
    us_center_subscriber.shutdown();
    us_left_subscriber.shutdown();
    us_right_subscriber.shutdown();
    imu_subscriber.shutdown();
<<<<<<< HEAD

    // Possible error - the following seems to shutdown all subscribers not just those from simulation

    for (map<string,ros::Subscriber>::iterator it=status_subscribers.begin(); it!=status_subscribers.end(); ++it) 
      {
	qApp->processEvents(QEventLoop::ExcludeUserInputEvents);
	it->second.shutdown();
      }
    status_subscribers.clear();

    for (map<string,ros::Subscriber>::iterator it=obstacle_subscribers.begin(); it!=obstacle_subscribers.end(); ++it) 
      {
	qApp->processEvents(QEventLoop::ExcludeUserInputEvents);
	it->second.shutdown();
      }

    obstacle_subscribers.clear();
    score_subscriber.shutdown();
    simulation_timer_subscriber.shutdown();
    camera_subscriber.shutdown();

    emit sendInfoLogMessage("Shutting down publishers...");

    for (map<string,ros::Publisher>::iterator it=control_mode_publishers.begin(); it!=control_mode_publishers.end(); ++it)
      {
	qApp->processEvents(QEventLoop::ExcludeUserInputEvents);
	it->second.shutdown();
      }
    control_mode_publishers.clear();

    return_msg += sim_mgr.stopGazeboClient();
    qApp->processEvents(QEventLoop::ExcludeUserInputEvents);
    return_msg += "<br>";
    return_msg += sim_mgr.stopGazeboServer();
    qApp->processEvents(QEventLoop::ExcludeUserInputEvents);
    emit sendInfoLogMessage(return_msg);
=======
    for (map<string,ros::Subscriber>::iterator it=obstacle_subscribers.begin(); it!=obstacle_subscribers.end(); ++it) it->second.shutdown();

    obstacle_subscribers.clear();
    for (map<string,ros::Subscriber>::iterator it=targetPickUpSubscribers.begin(); it!=targetPickUpSubscribers.end(); ++it) it->second.shutdown();
    targetPickUpSubscribers.clear();
    for (map<string,ros::Subscriber>::iterator it=targetDropOffSubscribers.begin(); it!=targetDropOffSubscribers.end(); ++it) it->second.shutdown();
    targetDropOffSubscribers.clear();
    camera_subscriber.shutdown();

    displayLogMessage("Shutting down publishers...");

    for (map<string,ros::Publisher>::iterator it=control_mode_publishers.begin(); it!=control_mode_publishers.end(); ++it) it->second.shutdown();
    control_mode_publishers.clear();

    return_msg += sim_mgr.stopGazeboClient();
    return_msg += "<br>";
    return_msg += sim_mgr.stopGazeboServer();
    displayLogMessage(return_msg);
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea

    ui.visualize_simulation_button->setEnabled(false);
    ui.build_simulation_button->setEnabled(true);
    ui.clear_simulation_button->setEnabled(false);
    display_sim_visualization = false;


    ui.build_simulation_button->setStyleSheet("color: white; border:1px solid white;");
    ui.visualize_simulation_button->setStyleSheet("color: grey; border:2px solid grey;");
    ui.clear_simulation_button->setStyleSheet("color: grey; border:2px solid grey;");

    // Clear the task status values
<<<<<<< HEAD
    obstacle_call_count = 0;
    emit updateObstacleCallCount("<font color='white'>0</font>");
    emit updateNumberOfTagsCollected("<font color='white'>0</font>");
    emit updateNumberOfSatellites("<font color='white'>Number of GPS Satellites: ---</font>");

    // reset the simulation timer variables
    ui.simulation_timer_combo_box->setEnabled(true);
    ui.simulation_timer_combo_box->setStyleSheet("color: white; border:1px solid white; padding: 1px 0px 1px 3px");
    ui.simulationTimerStartLabel->setText("<font color='white'>---</font>");
    ui.simulationTimerStopLabel->setText("<font color='white'>---</font>");
    ui.currentSimulationTimeLabel->setText("<font color='white'>---</font>");
    timer_start_time_in_seconds = 0.0;
    timer_stop_time_in_seconds = 0.0;
    current_simulated_time_in_seconds = 0.0;
    last_current_time_update_in_seconds = 0.0;
    is_timer_on = false;
}
=======
    ui.num_targets_collected_label->setText("<font color='white'>0</font>");
    ui.num_targets_detected_label->setText("<font color='white'>0</font>");
    targetsPickedUp.clear();
    targetsDroppedOff.clear();
    obstacle_call_count = 0;
    emit updateObstacleCallCount("<font color='white'>0</font>");
 }
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea

void RoverGUIPlugin::visualizeSimulationButtonEventHandler()
{
    if (!sim_mgr.isGazeboServerRunning())
    {
<<<<<<< HEAD
        emit sendInfoLogMessage("Simulation is not running.");
=======
        displayLogMessage("Simulation is not running.");
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea

        return;
    }

    QString return_msg;
    // toggle visualize or not
    display_sim_visualization = !display_sim_visualization;

    if (display_sim_visualization)
    {
<<<<<<< HEAD
        emit sendInfoLogMessage("Visualizing simulation...");
=======
        displayLogMessage("Visualizing simulation...");
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea

        QProcess* sim_client_process = sim_mgr.startGazeboClient();
    }
    else
    {
<<<<<<< HEAD
        emit sendInfoLogMessage("Ending visualization...");

        return_msg = sim_mgr.stopGazeboClient();
        emit sendInfoLogMessage(return_msg);
=======
        displayLogMessage("Ending visualization...");

        return_msg = sim_mgr.stopGazeboClient();
        displayLogMessage(return_msg);
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
    }

}

QString RoverGUIPlugin::startROSJoyNode()
{
    if (!joy_process)
    {

        QString argument = "rosrun joy joy_node";

        joy_process = new QProcess();

        joy_process->start("sh", QStringList() << "-c" << argument);

       // joy_process->waitForStarted();

        return "Started the joystick node.";

    }
    else
    {
        return "The joystick node is already running.";
    }
}

QString RoverGUIPlugin::stopROSJoyNode()
{
   //return "Do nothing for debug";

    if (joy_process)
    {
        joy_process->terminate();
        joy_process->waitForFinished();
        delete joy_process;
        joy_process = NULL;

        return "Stopped the running joystick node.";

    }
    else
    {
        return "Tried to stop the joystick node but it isn't running.";
    }
}

QString RoverGUIPlugin::addUniformTargets()
{
    QProgressDialog progress_dialog;
<<<<<<< HEAD
    progress_dialog.setWindowTitle("Placing 256 Targets"); //256 original value
=======
    progress_dialog.setWindowTitle("Placing 256 Targets");
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
    progress_dialog.setCancelButton(NULL); // no cancel button
    progress_dialog.setWindowModality(Qt::ApplicationModal);
    progress_dialog.resize(500, 50);
    progress_dialog.setWindowFlags(progress_dialog.windowFlags() | Qt::WindowStaysOnTopHint);
    progress_dialog.show();

    QString output;

    float proposed_x;
    float proposed_y;

    // 256 piles of 1 tag

    // d is the distance from the center of the arena to the boundary minus the barrier clearance, i.e. the region where tags can be placed
    // is d - U(0,2d) where U(a,b) is a uniform distribition bounded by a and b.
    // (before checking for collisions including the collection disk at the center)
    float d = arena_dim/2.0-(barrier_clearance+target_cluster_size_1_clearance);

<<<<<<< HEAD
    progress_dialog.setValue(0.0);
    qApp->processEvents(QEventLoop::ExcludeUserInputEvents);

    for (int i = 0; i < 256; i++)
    {
        do
        {
            emit sendInfoLogMessage("Tried to place target "+QString::number(0)+" at " + QString::number(proposed_x) + " " + QString::number(proposed_y) + "...");
            proposed_x = d - ((float) rand()) / RAND_MAX*2*d;
            proposed_y = d - ((float) rand()) / RAND_MAX*2*d;
        }
        while (sim_mgr.isLocationOccupied(proposed_x, proposed_y, target_cluster_size_1_clearance));

        emit sendInfoLogMessage("<font color=green>Succeeded.</font>");
        output = sim_mgr.addModel(QString("at")+QString::number(0),  QString("at")+QString::number(i), proposed_x, proposed_y, 0, target_cluster_size_1_clearance);
        progress_dialog.setValue(i*100.0f/256);
        qApp->processEvents(QEventLoop::ExcludeUserInputEvents);
    }

    emit sendInfoLogMessage("Placed 256 single targets");
=======
    for (int i = 0; i < 256; i++)
    {
        qApp->processEvents(QEventLoop::ExcludeUserInputEvents);
        do
        {
            displayLogMessage("Tried to place target "+QString::number(i)+" at " + QString::number(proposed_x) + " " + QString::number(proposed_y) + "...");
            proposed_x = d - ((float) rand()) / RAND_MAX*2*d;
            proposed_y = d - ((float) rand()) / RAND_MAX*2*d;
       }
       while (sim_mgr.isLocationOccupied(proposed_x, proposed_y, target_cluster_size_1_clearance));
       displayLogMessage("<font color=green>Succeeded.</font>");

        output = sim_mgr.addModel(QString("at")+QString::number(i),  QString("at")+QString::number(i), proposed_x, proposed_y, 0, target_cluster_size_1_clearance);

       progress_dialog.setValue(i*100.0f/256);
       qApp->processEvents(QEventLoop::ExcludeUserInputEvents);
    }
    displayLogMessage("Placed 256 single targets");
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea

    return output;
}

QString RoverGUIPlugin::addClusteredTargets()
{
    QProgressDialog progress_dialog;
    progress_dialog.setWindowTitle("Placing 256 Targets into 4 Clusters (64 targets each)");
    progress_dialog.setCancelButton(NULL); // no cancel button
    progress_dialog.setWindowModality(Qt::ApplicationModal);
    progress_dialog.setWindowFlags(progress_dialog.windowFlags() | Qt::WindowStaysOnTopHint);
    progress_dialog.resize(500, 50);
    progress_dialog.show();

    QString output;

<<<<<<< HEAD
    float proposed_x, proposed_x2;
    float proposed_y, proposed_y2;

    float d = arena_dim/2.0-(barrier_clearance+target_cluster_size_64_clearance);
    int cube_index = 0;

    progress_dialog.setValue(0.0);
    qApp->processEvents(QEventLoop::ExcludeUserInputEvents);
=======
    float proposed_x;
    float proposed_y;

    float d = arena_dim/2.0-(barrier_clearance+target_cluster_size_64_clearance);
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea

    // Four piles of 64
    for (int i = 0; i < 4; i++)
    {
<<<<<<< HEAD
        do
        {
            emit sendInfoLogMessage("Tried to place cluster "+QString::number(i)+" at " + QString::number(proposed_x) + " " + QString::number(proposed_y));
=======
        // Keep GUI responsive
        qApp->processEvents(QEventLoop::ExcludeUserInputEvents);

        do
        {
            displayLogMessage("Tried to place cluster "+QString::number(i)+" at " + QString::number(proposed_x) + " " + QString::number(proposed_y));
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
            proposed_x = d - ((float) rand()) / RAND_MAX*2*d;
            proposed_y = d - ((float) rand()) / RAND_MAX*2*d;
        }
        while (sim_mgr.isLocationOccupied(proposed_x, proposed_y, target_cluster_size_64_clearance));
<<<<<<< HEAD

        proposed_y2 = proposed_y - (target_cluster_size_1_clearance * 8);

        for(int j = 0; j < 8; j++) {
            proposed_x2 = proposed_x - (target_cluster_size_1_clearance * 8);

            for(int k = 0; k < 8; k++) {
                output += sim_mgr.addModel(QString("at")+QString::number(0),  QString("at")+QString::number(cube_index), proposed_x2, proposed_y2, 0, target_cluster_size_1_clearance);
                proposed_x2 += target_cluster_size_1_clearance;
                cube_index++;
                progress_dialog.setValue(cube_index*100.0f/256);
                qApp->processEvents(QEventLoop::ExcludeUserInputEvents);
            }

            proposed_y2 += target_cluster_size_1_clearance;
        }

        emit sendInfoLogMessage("<font color=green>Succeeded.</font>");
    }

    emit sendInfoLogMessage("Placed four clusters of 64 targets");
=======
        displayLogMessage("<font color=green>Succeeded.</font>");

        progress_dialog.setValue(i*100.0f/4);
        qApp->processEvents(QEventLoop::ExcludeUserInputEvents);

        output = sim_mgr.addModel(QString("atags64_")+QString::number(i), QString("atags64_")+QString::number(i), proposed_x, proposed_y, 0, target_cluster_size_64_clearance);
        displayLogMessage(output);
    }

    displayLogMessage("Placed four clusters of 64 targets");
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea

    return output;
}

QString RoverGUIPlugin::addPowerLawTargets()
{
    QProgressDialog progress_dialog;
    progress_dialog.setWindowTitle("Placing 256 Targets into 85 Clusters (Power Law pattern)");
    progress_dialog.setCancelButton(NULL); // no cancel button
    progress_dialog.setWindowModality(Qt::ApplicationModal);
    progress_dialog.setWindowFlags(progress_dialog.windowFlags() | Qt::WindowStaysOnTopHint);
    progress_dialog.resize(500, 50);
    progress_dialog.show();


    float total_number_of_clusters = 85;
    float clusters_placed = 0;
<<<<<<< HEAD
    int cube_index = 0;

    QString output = "";

    float proposed_x, proposed_x2;
    float proposed_y, proposed_y2;

    float d = arena_dim/2.0-(barrier_clearance+target_cluster_size_64_clearance);

    progress_dialog.setValue(0.0);
    qApp->processEvents(QEventLoop::ExcludeUserInputEvents);

    // One pile of 64
    do
    {
        emit sendInfoLogMessage("Tried to place cluster "+QString::number(clusters_placed)+" at " + QString::number(proposed_x) + " " + QString::number(proposed_y));
=======

    QString output = "";
    // One pile of 64

    float proposed_x;
    float proposed_y;

    float d = arena_dim/2.0-(barrier_clearance+target_cluster_size_64_clearance);

    do
    {
        displayLogMessage("Tried to place cluster "+QString::number(clusters_placed)+" at " + QString::number(proposed_x) + " " + QString::number(proposed_y));
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
        proposed_x = d - ((float) rand()) / RAND_MAX*2*d;
        proposed_y = d - ((float) rand()) / RAND_MAX*2*d;
    }
    while (sim_mgr.isLocationOccupied(proposed_x, proposed_y, target_cluster_size_64_clearance));
<<<<<<< HEAD

    proposed_y2 = proposed_y - (target_cluster_size_1_clearance * 8);

    for(int j = 0; j < 8; j++) {
        proposed_x2 = proposed_x - (target_cluster_size_1_clearance * 8);

        for(int k = 0; k < 8; k++) {
            output += sim_mgr.addModel(QString("at")+QString::number(0),  QString("at")+QString::number(cube_index), proposed_x2, proposed_y2, 0, target_cluster_size_1_clearance);
            proposed_x2 += target_cluster_size_1_clearance;
            cube_index++;
            progress_dialog.setValue(cube_index*100.0f/256);
            qApp->processEvents(QEventLoop::ExcludeUserInputEvents);
        }

        proposed_y2 += target_cluster_size_1_clearance;
    }

    emit sendInfoLogMessage("<font color=green>Succeeded.</font>");
=======
    displayLogMessage("<font color=green>Succeeded.</font>");

    progress_dialog.setValue(clusters_placed++*100.0f/total_number_of_clusters);
    output+= sim_mgr.addModel("atags64_0", "atags64_0", proposed_x, proposed_y, 0, target_cluster_size_64_clearance);

>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
    d = arena_dim/2.0-(barrier_clearance+target_cluster_size_16_clearance);

    // Four piles of 16
    for (int i = 0; i < 4; i++)
    {
<<<<<<< HEAD
=======
        qApp->processEvents(QEventLoop::ExcludeUserInputEvents);
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
        do
        {
            proposed_x = d - ((float) rand()) / RAND_MAX*2*d;
            proposed_y = d - ((float) rand()) / RAND_MAX*2*d;
<<<<<<< HEAD
            emit sendInfoLogMessage("Tried to place cluster "+QString::number(clusters_placed)+" at " + QString::number(proposed_x) + " " + QString::number(proposed_y));
        }
        while (sim_mgr.isLocationOccupied(proposed_x, proposed_y, target_cluster_size_16_clearance));

        proposed_y2 = proposed_y - (target_cluster_size_1_clearance * 4);

        for(int j = 0; j < 4; j++) {
            proposed_x2 = proposed_x - (target_cluster_size_1_clearance * 4);

            for(int k = 0; k < 4; k++) {
                output += sim_mgr.addModel(QString("at")+QString::number(0),  QString("at")+QString::number(cube_index), proposed_x2, proposed_y2, 0, target_cluster_size_1_clearance);
                proposed_x2 += target_cluster_size_1_clearance;
                cube_index++;
                progress_dialog.setValue(cube_index*100.0f/256);
                qApp->processEvents(QEventLoop::ExcludeUserInputEvents);
            }

            proposed_y2 += target_cluster_size_1_clearance;
        }

        emit sendInfoLogMessage("<font color=green>Succeeded.</font>");
=======
            displayLogMessage("Tried to place cluster "+QString::number(clusters_placed)+" at " + QString::number(proposed_x) + " " + QString::number(proposed_y));
        }
        while (sim_mgr.isLocationOccupied(proposed_x, proposed_y, target_cluster_size_16_clearance));
        displayLogMessage("<font color=green>Succeeded.</font>");


        progress_dialog.setValue(clusters_placed++*100.0f/total_number_of_clusters);
        qApp->processEvents(QEventLoop::ExcludeUserInputEvents);
        output+= sim_mgr.addModel(QString("atags16_")+QString::number(i), QString("atags16_")+QString::number(i), proposed_x, proposed_y, 0, target_cluster_size_64_clearance);
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
    }

    d = arena_dim/2.0-(barrier_clearance+target_cluster_size_4_clearance);

    // Sixteen piles of 4
    for (int i = 0; i < 16; i++)
    {
<<<<<<< HEAD
        do
        {
            emit sendInfoLogMessage("Tried to place cluster "+QString::number(clusters_placed)+" at " + QString::number(proposed_x) + " " + QString::number(proposed_y));
=======
        qApp->processEvents(QEventLoop::ExcludeUserInputEvents);
        do
        {
            displayLogMessage("Tried to place cluster "+QString::number(clusters_placed)+" at " + QString::number(proposed_x) + " " + QString::number(proposed_y));
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
            proposed_x = d - ((float) rand()) / RAND_MAX*2*d;
            proposed_y = d - ((float) rand()) / RAND_MAX*2*d;
        }
        while (sim_mgr.isLocationOccupied(proposed_x, proposed_y, target_cluster_size_4_clearance));
<<<<<<< HEAD

        proposed_y2 = proposed_y - (target_cluster_size_1_clearance * 2);

        for(int j = 0; j < 2; j++) {
            proposed_x2 = proposed_x - (target_cluster_size_1_clearance * 2);

            for(int k = 0; k < 2; k++) {
                output += sim_mgr.addModel(QString("at")+QString::number(0),  QString("at")+QString::number(cube_index), proposed_x2, proposed_y2, 0, target_cluster_size_1_clearance);
                proposed_x2 += target_cluster_size_1_clearance;
                cube_index++;
                progress_dialog.setValue(cube_index*100.0f/256);
                qApp->processEvents(QEventLoop::ExcludeUserInputEvents);
            }

            proposed_y2 += target_cluster_size_1_clearance;
        }

        emit sendInfoLogMessage("<font color=green>Succeeded.</font>");
=======
        displayLogMessage("<font color=green>Succeeded.</font>");

        progress_dialog.setValue(clusters_placed++*100.0f/total_number_of_clusters);
        qApp->processEvents(QEventLoop::ExcludeUserInputEvents);
        output+= sim_mgr.addModel(QString("atags4_")+QString::number(i), QString("atags4_")+QString::number(i), proposed_x, proposed_y, 0, target_cluster_size_4_clearance);
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
    }

    d = arena_dim/2.0-(barrier_clearance+target_cluster_size_1_clearance);

    // Sixty-four piles of 1 (using tags 192 through 255 to avoid duplication with piles above)
    for (int i = 192; i < 256; i++)
    {
<<<<<<< HEAD
        do
        {
            emit sendInfoLogMessage("Tried to place target "+QString::number(clusters_placed)+" at " + QString::number(proposed_x) + " " + QString::number(proposed_y));
=======
        qApp->processEvents(QEventLoop::ExcludeUserInputEvents);
        do
        {
            displayLogMessage("Tried to place target "+QString::number(clusters_placed)+" at " + QString::number(proposed_x) + " " + QString::number(proposed_y));
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
            proposed_x = d - ((float) rand()) / RAND_MAX*2*d;
            proposed_y = d - ((float) rand()) / RAND_MAX*2*d;
        }
        while (sim_mgr.isLocationOccupied(proposed_x, proposed_y, target_cluster_size_1_clearance));
<<<<<<< HEAD

        progress_dialog.setValue(i*100.0f/256);
        qApp->processEvents(QEventLoop::ExcludeUserInputEvents);
        emit sendInfoLogMessage("<font color=green>Succeeded.</font>");
        output+= sim_mgr.addModel(QString("at")+QString::number(0), QString("at")+QString::number(i), proposed_x, proposed_y, 0, target_cluster_size_1_clearance);
=======
        displayLogMessage("<font color=green>Succeeded.</font>");

        progress_dialog.setValue(clusters_placed++*100.0f/total_number_of_clusters);
        qApp->processEvents(QEventLoop::ExcludeUserInputEvents);
        output+= sim_mgr.addModel(QString("at")+QString::number(i), QString("at")+QString::number(i), proposed_x, proposed_y, 0, target_cluster_size_1_clearance);
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
    }

    return output;
}

// Add a cinder block wall to the simulation
QString RoverGUIPlugin::addFinalsWalls()
{
    QProgressDialog progress_dialog;
    progress_dialog.setWindowTitle("Placing Barriers");
    progress_dialog.setCancelButton(NULL); // no cancel button
    progress_dialog.setWindowModality(Qt::ApplicationModal);
    progress_dialog.setWindowFlags(progress_dialog.windowFlags() | Qt::WindowStaysOnTopHint);
    progress_dialog.resize(500, 50);
    progress_dialog.show();

    QString output;

    // Setting wall clearance to zero - radius of a wall does not make sense. Barrier clearance values ensure models are not placed on the walls.
    output += sim_mgr.addModel("barrier_final_round", "Barrier_West", -arena_dim/2, 0, 0, 0 );
    progress_dialog.setValue(1*100.0f/4);
    qApp->processEvents(QEventLoop::ExcludeUserInputEvents);

   output += sim_mgr.addModel("barrier_final_round", "Barrier_North", 0, -arena_dim/2, 0, 0, 0, M_PI/2, 0);
       progress_dialog.setValue(2*100.0f/4);
       qApp->processEvents(QEventLoop::ExcludeUserInputEvents);

   output += sim_mgr.addModel("barrier_final_round", "Barrier_East", arena_dim/2, 0, 0, 0 );
       progress_dialog.setValue(3*100.0f/4);
       qApp->processEvents(QEventLoop::ExcludeUserInputEvents);

   output += sim_mgr.addModel("barrier_final_round", "Barrier_South", 0, arena_dim/2, 0, 0, 0, M_PI/2, 0);
       progress_dialog.setValue(4*100.0f/4);
       qApp->processEvents(QEventLoop::ExcludeUserInputEvents);

   return output;
}

QString RoverGUIPlugin::addPrelimsWalls()
{
    QProgressDialog progress_dialog;
    progress_dialog.setWindowTitle("Placing Barriers");
    progress_dialog.setCancelButton(NULL); // no cancel button
    progress_dialog.setWindowModality(Qt::ApplicationModal);
    progress_dialog.setWindowFlags(progress_dialog.windowFlags() | Qt::WindowStaysOnTopHint);
    progress_dialog.resize(500, 50);
    progress_dialog.show();

    // Setting wall clearance to zero - radius of a wall does not make sense. Barrier clearance values ensure models are not placed on the walls.

   QString output;
   output += sim_mgr.addModel("barrier_prelim_round", "Barrier_West", -arena_dim/2, 0, 0, 0 );
   progress_dialog.setValue(1*100.0f/4);
   qApp->processEvents(QEventLoop::ExcludeUserInputEvents);

   output += sim_mgr.addModel("barrier_prelim_round", "Barrier_North", 0, -arena_dim/2, 0, 0, 0, M_PI/2, 0);
   progress_dialog.setValue(2*100.0f/4);
   qApp->processEvents(QEventLoop::ExcludeUserInputEvents);

   output += sim_mgr.addModel("barrier_prelim_round", "Barrier_East", arena_dim/2, 0, 0, 0 );
   progress_dialog.setValue(3*100.0f/4);
   qApp->processEvents(QEventLoop::ExcludeUserInputEvents);

   output += sim_mgr.addModel("barrier_prelim_round", "Barrier_South", 0, arena_dim/2, 0, 0, 0, M_PI/2, 0);
   progress_dialog.setValue(4*100.0f/4);
   qApp->processEvents(QEventLoop::ExcludeUserInputEvents);

   return output;
}

<<<<<<< HEAD
=======
int RoverGUIPlugin::targetDetect(const sensor_msgs::ImageConstPtr& rawImage) {

    cv_bridge::CvImagePtr cvImage;

	//Convert from MONO8 to BGR8
	//TODO: consider whether we should let the camera publish as BGR8 and skip this conversion
    try {
        cvImage = cv_bridge::toCvCopy(rawImage); //, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", rawImage->encoding.c_str());
        return -1;
    }

    //Create Mat image for processing
    cv::Mat matImage = cvImage->image;
    cv::cvtColor(matImage, matImage, cv::COLOR_BGR2GRAY);

    //Force greyscale and force image size.  This is only for Gazebo.
    //TODO: fix model so Gazebo publishes the correct format
    //TODO: if Mat is only used here, why not use the cvImage format here and skip the Mat image completely?
    if (matImage.cols != 320 && matImage.rows != 240) {
        cv::resize(matImage, matImage, cv::Size(320, 240), cv::INTER_LINEAR);
    }

    //Copy all image data into an array that AprilTag library expects
    image_u8_t *im = copy_image_data_into_u8_container(	matImage.cols, 
							matImage.rows, 
							(uint8_t *) matImage.data, 
							matImage.step);

    //Detect AprilTags
    zarray_t *detections = apriltag_detector_detect(td, im);
    
    //Check result for valid tag
    for (int i = 0; i < zarray_size(detections); i++) {
	    apriltag_detection_t *det;
	    zarray_get(detections, i, &det);
	
	    int tag = det->id;
	    
	    //Return first tag that has not been collected
	    if (targetsDroppedOff.count(tag) == 0){
			return tag;
		}
	}
	
	return -1;
}

image_u8_t* RoverGUIPlugin::copy_image_data_into_u8_container(int width, int height, uint8_t *rgb, int stride) {
    for (int y = 0; y < u8_image->height; y++) {
        for (int x = 0; x < u8_image->width; x++) {
            u8_image->buf[y * u8_image->stride + x] = rgb[y * stride + x + 0];
        }
    }
    return u8_image;
}

>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
void RoverGUIPlugin::checkAndRepositionRover(QString rover_name, float x, float y)
{
    // Currently disabled.
    return;

    float arena_dim = 20;

    if (x < -arena_dim/2)
    {
        float duration = 10; //seconds
        float x_comp, y_comp, z_comp;
        x_comp =
        z_comp = 0;
        y_comp = 0;
<<<<<<< HEAD
        emit sendInfoLogMessage("Moving rover back into the arena");
        QString return_msg = sim_mgr.moveRover(rover_name, x_comp, y, 0);
        emit sendInfoLogMessage(return_msg);
=======
        displayLogMessage("Moving rover back into the arena");
        QString return_msg = sim_mgr.moveRover(rover_name, x_comp, y, 0);
        displayLogMessage(return_msg);
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
    }
}

void RoverGUIPlugin::readRoverModelXML(QString path)
{
    ifstream model_file;
    model_file.open(path.toStdString(), ios::in);
    if (model_file.is_open())
<<<<<<< HEAD
        emit sendInfoLogMessage("Read model file at " + path );
    else
    {
        emit sendInfoLogMessage(QString::fromStdString(selected_rover_name) + " appears to be a physical rover.");
=======
        displayLogMessage("Read model file at " + path );
    else
    {
        displayLogMessage(QString::fromStdString(selected_rover_name) + " appears to be a physical rover.");
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
        return;
    }

    ptree property_tree;
    read_xml(model_file, property_tree);

    BOOST_FOREACH( ptree::value_type const& v, property_tree.get_child("sdf.model") )
    {
        if (v.first == "link")
        {
            BOOST_FOREACH( ptree::value_type const& w, v.second )
            {
                if (w.first == "sensor")
                {
                    BOOST_FOREACH( ptree::value_type const& x, w.second )
                    {
                        if ( x.first == "ray" )
                        {
                            BOOST_FOREACH( ptree::value_type const& y, x.second.get_child("scan.horizontal") )
                            {
                                if (y.first == "samples")
                                {
                                    //ui.sonar_horz_res->setText( QString::fromStdString(y.second.data()) );
                                }
                                else if (y.first == "resolution")
                                {
                                    ui.sonar_horz_res->setText( QString::fromStdString(y.second.data()) );
                                }
                                else if (y.first == "min_angle")
                                {
                                    ui.sonar_min_angle->setText( QString::fromStdString(y.second.data()) );
                                }
                                else if (y.first == "max_angle")
                                {
                                    ui.sonar_max_angle->setText( QString::fromStdString(y.second.data()) );
                                }
                            }

                            BOOST_FOREACH( ptree::value_type const& y, x.second.get_child("range") )
                            {
                                if (y.first == "min")
                                {
                                    ui.sonar_min->setText( QString::fromStdString(y.second.data()) );
                                }
                                else if (y.first == "max")
                                {
                                    ui.sonar_max->setText( QString::fromStdString(y.second.data()) );
                                }
                                else if (y.first == "resolution")
                                {
                                    ui.sonar_range_res->setText( QString::fromStdString(y.second.data()) );
                                }
                            }
                        }
                        else if ( x.first == "plugin" )
                        {

                            BOOST_FOREACH( ptree::value_type const& y, x.second )
                            {
                                if (y.first == "gaussianNoise")
                                {
                                    ui.sonar_gaussian_noise->setText( QString::fromStdString(y.second.data()) );
                                }
                            }
                        }
                        else if ( x.first == "camera" )
                        {
                            BOOST_FOREACH( ptree::value_type const& x, w.second )
                            {
                                if (x.first == "update_rate")
                                ui.camera_update_rate->setText( QString::fromStdString(x.second.data()) );
                            }

                            BOOST_FOREACH( ptree::value_type const& y, x.second )
                            {
                                if (y.first == "noise")
                                {
                                    BOOST_FOREACH( ptree::value_type const& z, y.second )
                                    {
                                         if (z.first == "mean") ui.camera_noise_mean->setText( QString::fromStdString(z.second.data()) );
                                         else if (z.first == "stddev") ui.camera_noise_stdev->setText( QString::fromStdString(z.second.data()) );
                                    }
                                }
                                else if (y.first == "image")
                                {
                                    BOOST_FOREACH( ptree::value_type const& z, y.second )
                                    {
                                         if (z.first == "width") ui.camera_width->setText( QString::fromStdString(z.second.data()) );
                                         else if (z.first == "height") ui.camera_height->setText( QString::fromStdString(z.second.data()) );
                                         else if (z.first == "format") ui.camera_format->setText( QString::fromStdString(z.second.data()) );
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        else if (v.first == "plugin")
        {

            BOOST_FOREACH( ptree::value_type const& w, v.second.get_child("<xmlattr>"))
            {
                 if (w.first == "name")
                    if (w.second.data() == "imu_sim")
                    {
                        BOOST_FOREACH( ptree::value_type const& x, v.second)
                        {
                            if (x.first == "updateRate") ui.imu_update_rate->setText(QString::fromStdString(x.second.data()));
                            else if (x.first == "rpyOffsets") ui.imu_rpy_offsets->setText(QString::fromStdString(x.second.data()));
                            else if (x.first == "gaussianNoise") ui.imu_noise->setText(QString::fromStdString(x.second.data()));
                            else if (x.first == "accelDrift") ui.imu_accel_drift->setText(QString::fromStdString(x.second.data()));
                            else if (x.first == "accelGaussianNoise") ui.imu_accel_noise->setText(QString::fromStdString(x.second.data()));
                            else if (x.first == "rateDrift") ui.imu_rate_drift->setText(QString::fromStdString(x.second.data()));
                            else if (x.first == "rateGaussianNoise") ui.imu_rate_noise->setText(QString::fromStdString(x.second.data()));
                            else if (x.first == "headingDrift") ui.imu_heading_drift->setText(QString::fromStdString(x.second.data()));
                            else if (x.first == "headingGaussianNoise") ui.imu_heading_noise->setText(QString::fromStdString(x.second.data()));
                        }
                    }
                 else if (w.second.data() == "gps_sim")
                    {
                        BOOST_FOREACH( ptree::value_type const& x, v.second)
                        {
                             if (x.first == "updateRate") ui.gps_update_rate->setText(QString::fromStdString(x.second.data()));
                             else if (x.first == "referenceLatitude") ui.gps_ref_lat->setText(QString::fromStdString(x.second.data()));
                             else if (x.first == "referenceLongitude") ui.gps_ref_long->setText(QString::fromStdString(x.second.data()));
                             else if (x.first == "referenceAltitude") ui.gps_ref_alt->setText(QString::fromStdString(x.second.data()));
                             else if (x.first == "referenceHeading") ui.gps_ref_heading->setText(QString::fromStdString(x.second.data()));
                             else if (x.first == "drift") ui.gps_drift->setText(QString::fromStdString(x.second.data()));
                             else if (x.first == "driftFrequency") ui.gps_drift_freq->setText(QString::fromStdString(x.second.data()));
                             else if (x.first == "gaussianNoise") ui.gps_noise->setText(QString::fromStdString(x.second.data()));

                        }
                    }
                }

            }
        else
        {

        }
    }

//    QDomElement root = xml_doc.documentElement();

//    QString gps_reference_lat = root.attribute("referenceLatitude");
//    QString gps_reference_long = root.attribute("referenceLongitude");
//    QString gps_reference_heading = root.attribute("referenceHeading");
//    QString gps_reference_altitude = root.attribute("referenceAltitude");
//    QString gps_offset = root.attribute("offset");
//    QString gps_drift = root.attribute("drift");
//    QString gps_drift_frequency = root.attribute("driftFrequency");
//    QString gps_gaussian_noise = root.attribute("gaussianNoise");

//    QString imu_update_rate = root.attribute("updateRate");
//    QString imu_rpy_offsets = root.attribute("rpyOffsets");
//    QString imu_gaussian_noise = root.attribute("gaussianNoise");
//    QString imu_accel_drift = root.attribute("accelDrift");
//    QString imu_accel_gaussian_noise = root.attribute("accelGaussianNoise");
//    QString imu_rate_drift = root.attribute("rateDrift");
//    QString imu_rate_gaussian_noise = root.attribute("rateGaussianNoise");
//    QString imu_heading_drift = root.attribute("headingDrift");
//    QString imu_heading_gaussian_noise = root.attribute("headingGaussianNoise");

//    QString camera_update_rate = root.attribute("update_rate");
//    QString camera_horizontal_fov = root.attribute("horizontal_fov");
//    QString camera_width = root.attribute("width");
//    QString camera_height = root.attribute("height");
//    QString camera_format = root.attribute("format");
//    QString camera_clip_near = root.attribute("near");
//    QString camera_clip_far = root.attribute("far");
//    QString camera_noise_type = root.attribute("type");
//    QString camera_noise_mean = root.attribute("mean");
//    QString camera_noise_stddev = root.attribute("stddev");

//    QString sonar_noise_mean = root.attribute("samples");
//    QString sonar_horz_resolution = root.attribute("resolution");
//    QString sonar_min_angle = root.attribute("min_angle");
//    QString sonar_max_angle = root.attribute("max_angle");
//    QString sonar_min = root.attribute("min");
//    QString sonar_max = root.attribute("max");
//    QString sonar_range_resolution = root.attribute("resolution");

    //cout << "GPS Ref. Lat. " << gps_reference_lat.toStdString() << endl;

}

void RoverGUIPlugin::gazeboServerFinishedEventHandler()
{

<<<<<<< HEAD
    emit sendInfoLogMessage("Gazebo client exited");
=======
    displayLogMessage("Gazebo client exited");
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea

    ui.visualize_simulation_button->setEnabled(false);
    ui.clear_simulation_button->setEnabled(false);
    ui.build_simulation_button->setEnabled(true);

    ui.visualize_simulation_button->setStyleSheet("color: grey; border:2px solid grey;");
    ui.clear_simulation_button->setStyleSheet("color: grey; border:2px solid grey;");
    ui.build_simulation_button->setStyleSheet("color: white; border:1px solid white;");
}

bool RoverGUIPlugin::eventFilter(QObject *target, QEvent *event)
{
<<<<<<< HEAD
    sensor_msgs::Joy joy_msg;
    joy_msg.axes = {0.0,0.0,0.0,0.0,0.0,0.0};
    
=======
    geometry_msgs::Twist standardized_joy_msg;
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
    if (joystick_publisher)
    {

    if (event->type() == QEvent::KeyPress)
    {
        QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);

            bool direction_key = true;

<<<<<<< HEAD
            float speed = 1;
=======
            float speed = 0.5;
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea

            // displayLogMessage("Key press");

            switch( keyEvent->key() )
            {
            case Qt::Key_I:
<<<<<<< HEAD
                joy_msg.axes[4] = speed;
                ui.joy_lcd_drive_forward->display(speed);
                break;
            case Qt::Key_K:
                joy_msg.axes[4] = -speed;
                ui.joy_lcd_drive_back->display(speed);
                break;
            case Qt::Key_J:
                joy_msg.axes[3] = speed;
                ui.joy_lcd_drive_left->display(speed);
                break;
            case Qt::Key_L:
                joy_msg.axes[3] = -speed;
                ui.joy_lcd_drive_right->display(speed);
=======
                standardized_joy_msg.linear.x = speed;
                ui.joy_lcd_forward->display(speed);
                break;
            case Qt::Key_K:
                standardized_joy_msg.linear.x = -speed;
                ui.joy_lcd_back->display(speed);
                break;
            case Qt::Key_J:
                standardized_joy_msg.angular.z = speed;
                ui.joy_lcd_left->display(speed);
                break;
            case Qt::Key_L:
                standardized_joy_msg.angular.z = -speed;
                ui.joy_lcd_right->display(speed);
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
                break;
            default:
                // Not a direction key so ignore
                direction_key = false;
            }

            if (direction_key )
            {
<<<<<<< HEAD
                joystick_publisher.publish(joy_msg);
=======
                joystick_publisher.publish(standardized_joy_msg);
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
                return true;
            }
        }

        // Stop the rover when key is released
        if (event->type() == QEvent::KeyRelease)
        {
            QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);

            // Don't process auto repeat release events. Just the actual key release.
            if (keyEvent->isAutoRepeat())
            {
                // displayLogMessage("Ignoring auto repeat release.");
                return rqt_gui_cpp::Plugin::eventFilter(target, event);
            }

            // displayLogMessage("Key release");

            if (keyEvent->key() == Qt::Key_I || keyEvent->key() == Qt::Key_J || keyEvent->key() == Qt::Key_K || keyEvent->key() == Qt::Key_L )
            {
<<<<<<< HEAD
                joy_msg.axes[4] = 0;
                joy_msg.axes[3] = 0;
                ui.joy_lcd_drive_forward->display(0);
                ui.joy_lcd_drive_back->display(0);
                ui.joy_lcd_drive_left->display(0);
                ui.joy_lcd_drive_right->display(0);

                joystick_publisher.publish(joy_msg);
=======
                standardized_joy_msg.linear.x = 0;
                standardized_joy_msg.angular.z = 0;
                ui.joy_lcd_forward->display(0);
                ui.joy_lcd_back->display(0);
                ui.joy_lcd_left->display(0);
                ui.joy_lcd_right->display(0);

                joystick_publisher.publish(standardized_joy_msg);
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
                return true;

            }
            else
            {
                return rqt_gui_cpp::Plugin::eventFilter(target, event);
            }
        }
    }
        // Pass on the event since it wasn't handled by us
    return rqt_gui_cpp::Plugin::eventFilter(target, event);
}

<<<<<<< HEAD
void RoverGUIPlugin::receiveInfoLogMessage(QString msg)
{
    displayInfoLogMessage(msg);
}


void RoverGUIPlugin::receiveDiagLogMessage(QString msg)
{
    displayDiagLogMessage(msg);
}

void RoverGUIPlugin::infoLogMessageEventHandler(const ros::MessageEvent<std_msgs::String const>& event)
{
    const std::string& publisher_name = event.getPublisherName();
    const ros::M_string& header = event.getConnectionHeader();
    ros::Time receipt_time = event.getReceiptTime();

    const boost::shared_ptr<const std_msgs::String> msg = event.getMessage();

    string log_msg = msg->data;

    emit sendInfoLogMessage(QString::fromStdString(publisher_name)
                           + " <font color=Lime size=1>"
                           + QString::fromStdString(log_msg)
                           + "</font>");
}

void RoverGUIPlugin::diagLogMessageEventHandler(const ros::MessageEvent<std_msgs::String const>& event)
{
    const std::string& publisher_name = event.getPublisherName();
    const ros::M_string& header = event.getConnectionHeader();
    ros::Time receipt_time = event.getReceiptTime();

    const boost::shared_ptr<const std_msgs::String> msg = event.getMessage();

    string log_msg = msg->data;

    emit sendDiagLogMessage(QString::fromStdString(log_msg));
}

void RoverGUIPlugin::overrideNumRoversCheckboxToggledEventHandler(bool checked)
{
    ui.custom_num_rovers_combobox->setEnabled(checked);
    if (checked) ui.custom_num_rovers_combobox->setStyleSheet("color: white; border:1px solid white; padding: 1px 0px 1px 3px"); // The padding makes the item list color change work
    else ui.custom_num_rovers_combobox->setStyleSheet("color: grey; border:1px solid grey;");
}

// Slot used to update the GUI diagnostic data output. Ensures we update from the correct process.
void RoverGUIPlugin::receiveDiagsDataUpdate(QString rover_name, QString text, QColor colour)
{
    if (!diag_update_mutex.try_lock()) return;

    // Find the row in the rover list that corresponds to the rover that sent us the diagnostics message
    // this is just to make sure the diagnostic data is displayed in the row that matches the rover
    // it came from
    int row = 0; // declare here so we can use it to index into the rover_diags_list
    for(; row < ui.rover_list->count(); row++)
    {
        QListWidgetItem *item = ui.rover_list->item(row);

        // Extract rover name
        string rover_name_and_status = item->text().toStdString();

        // Rover names start at the begining of the rover name and status string and end at the first space
        size_t rover_name_length = rover_name_and_status.find_first_of(" ");
        string ui_rover_name = rover_name_and_status.substr(0, rover_name_length);
        if (ui_rover_name.compare(rover_name.toStdString())==0) break; // We found a rover with the right name
    }

    // Check the the rover was found in the rover list
    if (row >= ui.rover_list->count())
    {
        emit sendInfoLogMessage("Received diagnostic data from an unknown rover: " + rover_name);
        return;
    }

    // Update the UI - needs to happen in the UI thread
    QListWidgetItem *item = ui.rover_diags_list->item(row);

    // We don't want the user to interact with this display item so make non-selectable
    item->setFlags(item->flags() & ~Qt::ItemIsSelectable);

    // Set the text and colour
    item->setText(text);
    item->setTextColor(colour);

    diag_update_mutex.unlock();
}


=======
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
// Refocus on the main ui widget so the rover list doesn't start capturing key strokes making keyboard rover driving not work.
void RoverGUIPlugin::refocusKeyboardEventHandler()
{
    widget->setFocus();
}

<<<<<<< HEAD
// Clean up memory when this object is deleted
RoverGUIPlugin::~RoverGUIPlugin()
{
    if (map_data) delete map_data;
    delete joystickGripperInterface;
}

=======
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
} // End namespace



PLUGINLIB_EXPORT_CLASS(rqt_rover_gui::RoverGUIPlugin, rqt_gui_cpp::Plugin)

