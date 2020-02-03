#include <ros/ros.h>
#include "trajectory_fitting/TargetPos.h"
#include "trajectory_fitting/Coefficients.h"
#include <visualization_msgs/Marker.h>
#include <cmath>

ros::Publisher marker_pub;
visualization_msgs::Marker points;
int forward_proj;

void CoefficientsCallback(const trajectory_fitting::Coefficients::ConstPtr& msg) {

    double t, t_i;
    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = "/my_frame";
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = "points_and_lines";
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;


    line_strip.id = 1;


    line_strip.type = visualization_msgs::Marker::LINE_STRIP;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.1;


    // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;


    t_i = ros::Time::now().toSec() - msg->offset;
    t = t_i;
    for(int i=0; i<forward_proj; i++) {

        double x = 0, y = 0, z = 0;

            for(int j=0; j<msg->coeffX.size(); j++) {
                x = x + msg->coeffX[j]*pow(t,j);
                y = y + msg->coeffY[j]*pow(t,j);
                z = z + msg->coeffZ[j]*pow(t,j);
            }

        geometry_msgs::Point p;
        p.x = x;
        p.y = y;
        p.z = z;

        line_strip.points.push_back(p);

        t = t_i + 0.1*i;
    }

    // marker_pub.publish(points);
    marker_pub.publish(line_strip);
}

void TargetPosCallback(const trajectory_fitting::TargetPos::ConstPtr& msg) {

    geometry_msgs::Point q;
    q.x = msg->position.point.x;
    q.y = msg->position.point.y;
    q.z = msg->position.point.z;

    points.header.frame_id = "/my_frame";
    points.header.stamp = ros::Time::now();
    points.ns = "points_and_lines";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;



    points.id = 0;

    points.type = visualization_msgs::Marker::POINTS;



    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.5;
    points.scale.y = 0.5;

    // Points are green
    points.color.r = 1.0;
    points.color.a = 1.0;

//    if( points.points.size() < 500 ) 
    points.points.push_back(q);

    marker_pub.publish(points);
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "marker_publisher");
  ros::NodeHandle n("~");


  if(n.getParam("forward_projection", forward_proj))
      ROS_INFO("Found parameter: setting projection dimension to %d", forward_proj);
  else    {
      ROS_INFO("Can't find polynomial grade");
  }

  marker_pub = n.advertise<visualization_msgs::Marker>("/visualization_marker", 100);
  ros::Subscriber coeff_sub = n.subscribe("/trajectory_fitting/coefficients", 25, CoefficientsCallback);
  ros::Subscriber target_sub = n.subscribe("/trajectory_fitting/target_pos2", 25, TargetPosCallback);

  ros::spin();

  return 0;
}
