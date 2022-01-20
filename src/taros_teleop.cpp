/*
This node allows to take manual control of the cameleon robot at any time.
When the robot is not controlled manually, the node simply receives commands over the cmd and flipperVelocity topic and passes them over to the cameleon/cmd_vel 
When the operator presses manualOverrideButton, the cmd and flipperVelocity messages are discarted and the operator can drive the robot manually. 
*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

//for person follow service

/*joystick input parameters - which button causes the user to take control and axes that correspond to forward, turning and flipper speeds*/ 
int manualOverrideButton = 0;
int steeringResetButton = 9;
int wheelResetAxisA = 2;
int wheelResetAxisB = 5;
int speedCoefAxis = 7;
int linearResetButton = 10;
int linearAxis = 1;
int angularAxis = 0;

/*these constants determine how quickly the robot moves based on the joystick input*/ 
double wheelSteer = 0;
double controlRate = 15;
double linearGain = 0.5;
double steeringGain = 1.0;
double maxSteering = 1.57;
double maxLinear = 1.0;
double obstacleInfluence = 1.0;
bool emergencyBreak = false; 
double speedCoef = 1;

/*listening to joystick, flipperVelocity and cmd topics and publishing commands to the cameleon ros driver*/
ros::Publisher vel_pub_;
ros::Subscriber joy_sub_;
ros::Subscriber status_sub_;
ros::Subscriber health_sub_;
ros::Subscriber cmd_sub_;
ros::Publisher gripper_pub; //used to load/unload the car
ros::Publisher reset_pub; //used to reset the wheels 

/*state variables - twist is the message that eventually gets to the ROS driver of the robot, other are obvious*/
geometry_msgs::Twist twist;
bool teleoperated = false;
double forwardSpeed = 0;
double forwardAcceleration= 0;
double steeringSpeed = 0;
double steeringAngle = 0;

/*person follower variables*/
bool currentlyFollowing = false;
int personFollowerButton = 7;
ros::ServiceClient followerClient;

/*toggling actuators/car dropping*/
int actuatorButton = 6;
bool actuatorAllowLifting = true; //allow lifting the car -else only allow dropping it
bool actuatorLifting = true; //if closed or open. probably safer to drop first
ros::Time actuatorMovementStart(0); //time since button was pressed
ros::Duration actuatorPublishDuration(10.0); //how long to publish the close/open actuator message for
ros::Duration maxDeadTime(0.5); //how long to wait for system health 
ros::Time lastHealthReport; //how long to wait for system health 

void publishFollower(bool follow)
{
  //call rosservice to follower node
  /*lipraco_teleop::followPerson srv;
  srv.request.followPerson = follow;
  if(followerClient.call(srv))
    ROS_INFO("Teleop: following person");
  else
    ROS_ERROR("Teleop: unable to call service to start following");
  */
}

/*commands from higher-lever modules*/
void cmdCallback(const geometry_msgs::Twist::ConstPtr& cmd)
{
	/*if the robot is note teleoperated, form the twist command from the incoming cmd topic*/
	if (teleoperated == false){
		if((ros::Time::now() - lastHealthReport) < maxDeadTime){
			twist.linear.x  = cmd->linear.x;
			twist.linear.z  = cmd->linear.z;
			twist.angular.z = cmd->angular.z;
		}else{
			ROS_WARN("Not receiving diagnostics");
			twist.linear.x  = 0;
			twist.linear.z  = 0;
			twist.angular.z = 0;
		}
	}
}

/*receiving joystick data*/
void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{    
	/*if swiching modes from teleoperated to manual and vice versa, then clear velocities*/
	if (teleoperated != joy->buttons[manualOverrideButton]){
		forwardAcceleration = 0;
		steeringSpeed = 0;
		forwardSpeed = 0;
		twist.linear.x =  0;
		steeringAngle = wheelSteer;

		//cancel any person following
		publishFollower(false);
	}

	/*is it teleoperated?*/
	teleoperated = (joy->buttons[manualOverrideButton] == 1);

	if (teleoperated)
	{
		steeringSpeed = steeringGain*joy->axes[angularAxis];
		forwardAcceleration = linearGain*joy->axes[linearAxis];
		if (forwardAcceleration > 0){
			forwardAcceleration = forwardAcceleration - 0.4;
		        if (forwardAcceleration < 0) forwardAcceleration = 0;
		}
		if (forwardAcceleration < 0){
			forwardAcceleration = forwardAcceleration + 0.4; 
		        if (forwardAcceleration > 0) forwardAcceleration = 0;
		}
		printf("%f %f\n",forwardAcceleration,joy->axes[linearAxis]);
		if (joy->buttons[steeringResetButton]==1)
		{
			steeringSpeed = 0;
			steeringAngle = 0;
			twist.angular.z =  0;
		}
		if (joy->buttons[linearResetButton]==1)
		{
			forwardAcceleration = 0;
			forwardSpeed = 0;
			twist.linear.x =  0;
		}
		ROS_INFO( "Speed: %.3f Steering: %.3f Accel: %.3f SteerSpeed: %.3f",  twist.linear.x,twist.angular.z,forwardAcceleration,steeringSpeed);
	}

	//if user toggles person follower
	/*if(joy->buttons[personFollowerButton])
	{
		currentlyFollowing = !currentlyFollowing;

		if(currentlyFollowing)
			publishFollower(true);
		else
			publishFollower(false);
	}

	//dropping (and lifting) car
	if(joy->buttons[actuatorButton])
	{
		actuatorLifting = !actuatorLifting;
		actuatorMovementStart = ros::Time::now();
	}
	if (joy->axes[speedCoefAxis] == -1) speedCoef = fmax(0,speedCoef - 0.2);
	if (joy->axes[speedCoefAxis] == +1) speedCoef = fmin(1,speedCoef + 0.2);
	//reset stuck wheels
	if (joy->axes[wheelResetAxisA]== -1 && joy->axes[wheelResetAxisB]== -1)
	{
		std_msgs::Bool rst_msg;
		rst_msg.data = true;
		emergencyBreak = false;		
		reset_pub.publish(rst_msg);
	} else if (joy->axes[wheelResetAxisA]== -1)
	{
		emergencyBreak = true;		
	} 
	if (joy->axes[wheelResetAxisA] != -1){
		emergencyBreak = false;		
	}*/
}



int main(int argc, char** argv)
{
	ros::init(argc, argv, "cameleon_teleop");
	ros::NodeHandle nh;

	nh.param("axis_linear", linearAxis, 1);
	nh.param("axis_angular", angularAxis, 0);
	nh.param("manual_override_button", manualOverrideButton, 0);

	nh.param("steering_gain", steeringGain, 1.0);
	nh.param("linear_gain", linearGain, 1.0);

	nh.param("max_steering", maxSteering, 1.57);
	nh.param("max_linear", maxLinear, 0.1);
	nh.param("control_rate", controlRate, 15.0);

	vel_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	joy_sub_ = nh.subscribe<sensor_msgs::Joy>("remote", 10, joyCallback);
	cmd_sub_ = nh.subscribe<geometry_msgs::Twist>("cmd", 10, cmdCallback);

	ros::Rate loopRate(controlRate);
	while (ros::ok()){
		if (teleoperated){
			forwardSpeed = forwardAcceleration;
			forwardSpeed = fmin(fmax(forwardSpeed,-maxLinear),maxLinear);
			twist.linear.x =  forwardSpeed;
			twist.linear.z =  0;

			steeringAngle = steeringSpeed;///controlRate; 
			steeringAngle = fmin(fmax(steeringAngle,-maxSteering),maxSteering);
			twist.angular.z =  -steeringAngle;

			/*std::cout <<	ros::Time::now() 	<< std::endl;
			  std::cout <<	actuatorMovementStart 	<< std::endl;
			  std::cout <<	actuatorPublishDuration << std::endl;*/
			if((ros::Time::now() - actuatorMovementStart) < actuatorPublishDuration)
			{
				if(actuatorLifting && actuatorAllowLifting)
				{
					printf("Actuator Lifting\n");
					//close actuators
					std_msgs::Bool load_msg;
					load_msg.data = true;
					gripper_pub.publish(load_msg);
				}
				else if(!actuatorLifting)
				{
					//open actuators
					printf("Actuator Releasing\n");
					std_msgs::Bool load_msg;
					load_msg.data = false;
					gripper_pub.publish(load_msg);
				}
			}
		}
		if (emergencyBreak){
			twist.linear.x  = 0;
			twist.linear.z  = 5;
		}
		twist.linear.x  = twist.linear.x*speedCoef;
		vel_pub_.publish(twist);
		ros::spinOnce();
		loopRate.sleep();
	}
}
