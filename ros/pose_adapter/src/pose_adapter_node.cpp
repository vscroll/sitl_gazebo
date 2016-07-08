#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include <fmaros_msgs/PendulumPose.h>
#include "VehiclePendulumPose.pb.h"

#define PORT 14000

int main(int argc, char **argv)
{
  int sockfd,len;
  struct sockaddr_in addr;
  socklen_t  addr_len = sizeof(struct sockaddr_in);
  char buffer[4096];
  char buf[128];

  if((sockfd=socket(AF_INET,SOCK_DGRAM,0))<0){
	  perror ("socket");
	  exit(1);
  }

  bzero(&addr, sizeof(addr));
  addr.sin_family=AF_INET;
  addr.sin_port=htons(PORT);
  addr.sin_addr.s_addr=htonl(INADDR_ANY) ;
  if (bind(sockfd, (struct sockaddr *)&addr, sizeof(addr))<0){
	  perror("connect");
	  exit(1);
  }

  ros::init(argc, argv, "pose_adapter_node");

  ros::NodeHandle n;

  ros::Publisher pendulum_pub = n.advertise<fmaros_msgs::PendulumPose>("/FMA/Pendulum/Pose", 1000);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    fmaros_msgs::PendulumPose pose;

    ros::spinOnce();

    loop_rate.sleep();

    bzero(buffer,sizeof(buffer));
    len = recvfrom(sockfd,buffer,sizeof(buffer), 0 , (struct sockaddr *)&addr, &addr_len);

    if (len > 0) {
        vehiclependulum_msgs::msgs::VehiclePendulumPose pose_message_;

	pose_message_.ParseFromArray(buffer, len);
	int64_t time = pose_message_.time_usec();
	pendulum_msgs::msgs::PendulumPose pendulumpose = pose_message_.pendulum_pose();
	vehicle_msgs::msgs::VehiclePose vehiclepose =  pose_message_.vehicle_pose(0);

	pose.header.stamp.nsec = time;

	pose.position.x = pendulumpose.x();
	pose.position.y = pendulumpose.y();
	pose.position.z = pendulumpose.z();

	pose.velocity.x = pendulumpose.vx();
	pose.velocity.y = pendulumpose.vy();
	pose.velocity.z = pendulumpose.vz();

	pose.vel_acc.x = pendulumpose.acc_x();
	pose.vel_acc.y = pendulumpose.acc_y();
	pose.vel_acc.z = pendulumpose.acc_z();

	pendulum_pub.publish(pose);
    }
  }

  return 0;
}
