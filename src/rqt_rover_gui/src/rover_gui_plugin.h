/*!
 * \brief   This is the main class that creates and connects the GUI components.
 *          The rover_gui_plugin.ui file is generated by the UI editor in QTCreator and most GUI elements are delared there
 *          and accessed via the ui object.
 *          RoverGUIPlugin is a plugin class for the rqt system. It compiles into a shared library that rqt can use. The rover GUI can be used
 *          by selecting it from within rtq or by running rtq -s rtq_rover_gui
 *          RoverGUIPlugin is event driven. The events either come from the ROS system or from QT. Event handlers process these events and update
 *          the GUI or send commands to the rovers as needed.
 *          This class also interfaces with GazeboSimManager in order to manipulate models in simulation.
 *
 * \author  Matthew Fricke
 * \date    November 11th 2015
 * \todo    Code works properly.
 * \class   RoverGUIPlugin
 */

#ifndef ROVERGUIPLUGIN_H
#define ROVERGUIPLUGIN_H

#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

#include <rqt_gui_cpp/plugin.h>
#include <ui_rover_gui_plugin.h>
//#include <rqt_rover_gui/ui_rover_gui_plugin.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Image.h>
#include <ros/macros.h>
<<<<<<< HEAD
#include <rosgraph_msgs/Clock.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt8.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
=======
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt8.h>
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
#include <pluginlib/class_list_macros.h>
#include <QGraphicsView>
#include <QEvent>
#include <QKeyEvent>
<<<<<<< HEAD
#include <QListWidget> // Provides QListWidgetItem
#include <QProcess>
#include <map>
#include <set>
#include <mutex>
#include <ublox_msgs/NavSOL.h>
=======
#include <QProcess>

#include <map>
#include <set>
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea

//ROS msg types
//#include "rover_onboard_target_detection/ATag.h"
//#include "rover_onboard_target_detection/harvest.h"

#include <QWidget>
#include <QTimer>
#include <QLabel>

#include "GazeboSimManager.h"
<<<<<<< HEAD
#include "JoystickGripperInterface.h"


// Forward declarations
class MapData;

using namespace std;


// RoverStaus holds status messages from rovers and time
// time they were received. The time is used to detect
// disconnected rovers. The status message can be any string
// in the past it has been used to display the team name
// in the GUI
struct RoverStatus {
    string status_msg;
    ros::Time timestamp;
};


=======

//AprilTag headers
#include "apriltag.h"
#include "tag36h11.h"
#include "tag36h10.h"
#include "tag36artoolkit.h"
#include "tag25h9.h"
#include "tag25h7.h"
#include "common/pnm.h"
#include "common/image_u8.h"
#include "common/zarray.h"
#include "common/getopt.h"

using namespace std;

>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
namespace rqt_rover_gui {

  class RoverGUIPlugin : public rqt_gui_cpp::Plugin
  {
    Q_OBJECT
      
  public:
<<<<<<< HEAD

    RoverGUIPlugin();
    ~RoverGUIPlugin();
=======
    RoverGUIPlugin();
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
    virtual void initPlugin(qt_gui_cpp::PluginContext& context);
    virtual void shutdownPlugin();
    virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
    virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);
<<<<<<< HEAD

=======
    
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
    bool eventFilter(QObject *target, QEvent *event);

    // Handles output from the joystick node
    QString startROSJoyNode();
    QString stopROSJoyNode();

<<<<<<< HEAD
    void statusEventHandler(const ros::MessageEvent<std_msgs::String const>& event);
=======
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
    void joyEventHandler(const sensor_msgs::Joy::ConstPtr& joy_msg);
    void cameraEventHandler(const sensor_msgs::ImageConstPtr& image);
    void EKFEventHandler(const ros::MessageEvent<const nav_msgs::Odometry> &event);
    void GPSEventHandler(const ros::MessageEvent<const nav_msgs::Odometry> &event);
<<<<<<< HEAD
    void GPSNavSolutionEventHandler(const ros::MessageEvent<const ublox_msgs::NavSOL> &event);
    void encoderEventHandler(const ros::MessageEvent<const nav_msgs::Odometry> &event);
    void obstacleEventHandler(const ros::MessageEvent<std_msgs::UInt8 const> &event);
    void scoreEventHandler(const ros::MessageEvent<std_msgs::String const> &event);
    void simulationTimerEventHandler(const rosgraph_msgs::Clock& msg);
    void diagnosticEventHandler(const ros::MessageEvent<std_msgs::Float32MultiArray const> &event);
=======
    void encoderEventHandler(const ros::MessageEvent<const nav_msgs::Odometry> &event);
    void targetPickUpEventHandler(const ros::MessageEvent<const sensor_msgs::Image> &event);
    void targetDropOffEventHandler(const ros::MessageEvent<const sensor_msgs::Image> &event);
    void obstacleEventHandler(const ros::MessageEvent<std_msgs::UInt8 const>& event);
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea

    void centerUSEventHandler(const sensor_msgs::Range::ConstPtr& msg);
    void leftUSEventHandler(const sensor_msgs::Range::ConstPtr& msg);
    void rightUSEventHandler(const sensor_msgs::Range::ConstPtr& msg);
    void IMUEventHandler(const sensor_msgs::Imu::ConstPtr& msg);

<<<<<<< HEAD
    void infoLogMessageEventHandler(const ros::MessageEvent<std_msgs::String const>& event);
    void diagLogMessageEventHandler(const ros::MessageEvent<std_msgs::String const>& event);


=======
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
    void addModelToGazebo();
    QString addPowerLawTargets();
    QString addUniformTargets();
    QString addClusteredTargets();
    QString addFinalsWalls();
    QString addPrelimsWalls();


   // void targetDetectedEventHandler( rover_onboard_target_detection::ATag tagInfo ); //rover_onboard_target_detection::ATag msg );

    void setupSubscribers();
    void setupPublishers();

    // Detect rovers that are broadcasting information
    set<string> findConnectedRovers();
<<<<<<< HEAD

  signals:

    void sendInfoLogMessage(QString); // log message updates need to be implemented as signals so they can be used in ROS event handlers.
    void sendDiagLogMessage(QString);    
    void sendDiagsDataUpdate(QString, QString, QColor); // Provide the item to update and the diags text and text color

    // Joystick output - Drive
    void joystickDriveForwardUpdate(double);
    void joystickDriveBackwardUpdate(double);
    void joystickDriveLeftUpdate(double);
    void joystickDriveRightUpdate(double);

    // Joystick GUI output - Gripper
    void joystickGripperWristUpUpdate(double);
    void joystickGripperWristDownUpdate(double);
    void joystickGripperFingersCloseUpdate(double);
    void joystickGripperFingersOpenUpdate(double);

    void updateObstacleCallCount(QString text);
    void updateNumberOfTagsCollected(QString text);
    void updateNumberOfSatellites(QString text);
    void allStopButtonSignal();
    void updateCurrentSimulationTimeLabel(QString text);

  private slots:

    void receiveDiagsDataUpdate(QString, QString, QColor);
    void receiveInfoLogMessage(QString);
    void receiveDiagLogMessage(QString);
=======
    
    //Image converter
	image_u8_t *copy_image_data_into_u8_container(int width, int height, uint8_t *rgb, int stride);

	//AprilTag detector
	int targetDetect(const sensor_msgs::ImageConstPtr& rawImage);

  signals:

    void joystickForwardUpdate(double);
    void joystickBackUpdate(double);
    void joystickLeftUpdate(double);
    void joystickRightUpdate(double);
    void updateObstacleCallCount(QString text);
    void updateLog(QString text);

  private slots:

>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
    void currentRoverChangedEventHandler(QListWidgetItem *current, QListWidgetItem *previous);
    void pollRoversTimerEventHandler();
    void GPSCheckboxToggledEventHandler(bool checked);
    void EKFCheckboxToggledEventHandler(bool checked);
    void encoderCheckboxToggledEventHandler(bool checked);
<<<<<<< HEAD
    void overrideNumRoversCheckboxToggledEventHandler(bool checked);

    void mapSelectionListItemChangedHandler(QListWidgetItem* changed_item);
    void mapAutoRadioButtonEventHandler(bool marked);
    void mapManualRadioButtonEventHandler(bool marked);
    void mapPopoutButtonEventHandler();
=======
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea

    void joystickRadioButtonEventHandler(bool marked);
    void autonomousRadioButtonEventHandler(bool marked);
    void allAutonomousButtonEventHandler();
    void allStopButtonEventHandler();
<<<<<<< HEAD
    void customWorldButtonEventHandler();
    void customWorldRadioButtonEventHandler(bool marked);
=======
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea

    void buildSimulationButtonEventHandler();
    void clearSimulationButtonEventHandler();
    void visualizeSimulationButtonEventHandler();
<<<<<<< HEAD
    void gazeboServerFinishedEventHandler();
    void displayInfoLogMessage(QString msg);
    void displayDiagLogMessage(QString msg);
=======
    void gazeboServerFinishedEventHandler();  
    void displayLogMessage(QString msg);
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea

    // Needed to refocus the keyboard events when the user clicks on the widget list
    // to the main widget so keyboard manual control is handled properly
    void refocusKeyboardEventHandler();

  private:

    void checkAndRepositionRover(QString rover_name, float x, float y);
    void readRoverModelXML(QString path);

<<<<<<< HEAD
    // ROS Publishers
    map<string,ros::Publisher> control_mode_publishers;
    ros::Publisher joystick_publisher;

    // ROS Subscribers
    ros::Subscriber joystick_subscriber;
    map<string,ros::Subscriber> encoder_subscribers;
    map<string,ros::Subscriber> gps_subscribers;
    map<string,ros::Subscriber> gps_nav_solution_subscribers;
    map<string,ros::Subscriber> ekf_subscribers;
    map<string,ros::Subscriber> rover_diagnostic_subscribers;
=======
    map<string,ros::Publisher> control_mode_publishers;
    ros::Publisher joystick_publisher;
    map<string,ros::Publisher> targetPickUpPublisher;
    map<string,ros::Publisher> targetDropOffPublisher;

    ros::Subscriber joystick_subscriber;
    map<string,ros::Subscriber> encoder_subscribers;
    map<string,ros::Subscriber> gps_subscribers;
    map<string,ros::Subscriber> ekf_subscribers;
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
    ros::Subscriber us_center_subscriber;
    ros::Subscriber us_left_subscriber;
    ros::Subscriber us_right_subscriber;
    ros::Subscriber imu_subscriber;
<<<<<<< HEAD
    ros::Subscriber info_log_subscriber;
    ros::Subscriber diag_log_subscriber;
    ros::Subscriber score_subscriber;
    ros::Subscriber simulation_timer_subscriber;

    map<string,ros::Subscriber> status_subscribers;
    map<string,ros::Subscriber> obstacle_subscribers;
=======

    map<string,ros::Subscriber> obstacle_subscribers;
    map<string,ros::Subscriber> targetDropOffSubscribers;
    map<string,ros::Subscriber> targetPickUpSubscribers;
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
    image_transport::Subscriber camera_subscriber;

    string selected_rover_name;
    set<string> rover_names;
    ros::NodeHandle nh;
    QWidget* widget;
    Ui::RoverGUI ui;

    QProcess* joy_process;
    QTimer* rover_poll_timer; // for rover polling

<<<<<<< HEAD
    QString info_log_messages;
    QString diag_log_messages;

    GazeboSimManager sim_mgr;

    map<string,int> rover_control_state;
    map<string,int> rover_numSV_state;
    map<string, RoverStatus> rover_statuses;

    float arena_dim; // in meters

    // simulation timer variables
    double current_simulated_time_in_seconds;
    double last_current_time_update_in_seconds;
    double timer_start_time_in_seconds;
    double timer_stop_time_in_seconds;
    bool is_timer_on;
    // helper functions for the simulation timer
    double getHours(double seconds);
    double getMinutes(double seconds);
    double getSeconds(double seconds);
=======
    QString log_messages;
    GazeboSimManager sim_mgr;

    map<string,int> rover_control_state;

    float arena_dim; // in meters

    map<string,int> targetsPickedUp;
    map<int,bool> targetsDroppedOff;
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea

    bool display_sim_visualization;

    // Object clearance. These values are used to quickly determine where objects can be placed int time simulation
    float target_cluster_size_64_clearance;
    float target_cluster_size_16_clearance;
    float target_cluster_size_4_clearance;
    float target_cluster_size_1_clearance;
    float rover_clearance;
    float collection_disk_clearance;
    float barrier_clearance;

    unsigned long obstacle_call_count;
<<<<<<< HEAD

    // Joystick commands to ROS gripper command interface
    // This is a singleton class. Deleting it while it is still receiving movement commands will cause a segfault.
    // Use changeRovers instead of recreating with a new rover name
    JoystickGripperInterface* joystickGripperInterface;

    // Amount of time between status messages that results in a rover disconnect
    ros::Duration disconnect_threshold;

    MapData* map_data;

    // Limit the length of log messages in characters to prevent slowdowns when lots of data is added
    size_t max_info_log_length;
    size_t max_diag_log_length;

    std::mutex diag_update_mutex;
=======
    
    //AprilTag objects
	apriltag_family_t *tf = NULL; //tag family
	apriltag_detector_t *td = NULL; //tag detector

	//Image container
	image_u8_t *u8_image = NULL;
	
	//AprilTag assigned to collection zone
	int collectionZoneID = 256;
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
  };
} // end namespace

#endif // ROVERGUIPLUGIN
