#include <ros/ros.h>

//ROS libraries
#include <message_filters/subscriber.h>
<<<<<<< HEAD
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
=======
#include <message_filters/time_synchronizer.h>
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea

//ROS messages
#include <std_msgs/UInt8.h>
#include <sensor_msgs/Range.h>

using namespace std;

//Globals
<<<<<<< HEAD
double collisionDistance = 0.6; //meters the ultrasonic detectors will flag obstacles
=======
double collisionDistance = 0.4; //meters the ultrasonic detectors will flag obstacles
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
string publishedName;
char host[128];

//Publishers
ros::Publisher obstaclePublish;

//Callback handlers
void sonarHandler(const sensor_msgs::Range::ConstPtr& sonarLeft, const sensor_msgs::Range::ConstPtr& sonarCenter, const sensor_msgs::Range::ConstPtr& sonarRight);

int main(int argc, char** argv) {
    gethostname(host, sizeof (host));
    string hostname(host);
    
    if (argc >= 2) {
        publishedName = argv[1];
        cout << "Welcome to the world of tomorrow " << publishedName << "! Obstacle module started." << endl;
    } else {
        publishedName = hostname;
        cout << "No name selected. Default is: " << publishedName << endl;
    }

    ros::init(argc, argv, (publishedName + "_OBSTACLE"));
    ros::NodeHandle oNH;
    
    obstaclePublish = oNH.advertise<std_msgs::UInt8>((publishedName + "/obstacle"), 10);
    
    message_filters::Subscriber<sensor_msgs::Range> sonarLeftSubscriber(oNH, (publishedName + "/sonarLeft"), 10);
    message_filters::Subscriber<sensor_msgs::Range> sonarCenterSubscriber(oNH, (publishedName + "/sonarCenter"), 10);
<<<<<<< HEAD
    message_filters::Subscriber<sensor_msgs::Range> sonarRightSubscriber(oNH, (publishedName + "/sonarRight"), 10);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Range, sensor_msgs::Range, sensor_msgs::Range> sonarSyncPolicy;

    message_filters::Synchronizer<sonarSyncPolicy> sonarSync(sonarSyncPolicy(10), sonarLeftSubscriber, sonarCenterSubscriber, sonarRightSubscriber);
    sonarSync.registerCallback(boost::bind(&sonarHandler, _1, _2, _3));
=======
    message_filters::Subscriber<sensor_msgs::Range> sonarRightSubscriber(oNH, (publishedName + "/sonarRight"), 10);	
    message_filters::TimeSynchronizer<sensor_msgs::Range, sensor_msgs::Range, sensor_msgs::Range> sonarSync(sonarLeftSubscriber, sonarCenterSubscriber, sonarRightSubscriber, 10);
	sonarSync.registerCallback(boost::bind(&sonarHandler, _1, _2, _3));
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea

    ros::spin();

    return EXIT_SUCCESS;
}

void sonarHandler(const sensor_msgs::Range::ConstPtr& sonarLeft, const sensor_msgs::Range::ConstPtr& sonarCenter, const sensor_msgs::Range::ConstPtr& sonarRight) {
	std_msgs::UInt8 obstacleMode;
	
	if ((sonarLeft->range > collisionDistance) && (sonarCenter->range > collisionDistance) && (sonarRight->range > collisionDistance)) {
		obstacleMode.data = 0; //no collision
	}
	else if ((sonarLeft->range > collisionDistance) && (sonarRight->range < collisionDistance)) {
		obstacleMode.data = 1; //collision on right side
	}
	else {
		obstacleMode.data = 2; //collision in front or on left side
	}
<<<<<<< HEAD
	if (sonarCenter->range < 0.10) //block in front of center unltrasound. original 0.12 - Abe
	{
		obstacleMode.data = 4;
	}
=======
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
	
        obstaclePublish.publish(obstacleMode);
}

