#include "ros/ros.h"
#include <iostream>
#include <deque>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include "geometry_msgs/PointStamped.h"
#include "trajectory_fitting/Coefficients.h"
#include <nav_msgs/Odometry.h>


class TrajectoryFitting     {
public:
    TrajectoryFitting();
    void callback(const geometry_msgs::PointStamped::ConstPtr& msg);
    void OdomCallback(const nav_msgs::Odometry::ConstPtr& odometry);

private:
    ros::NodeHandle node = ros::NodeHandle("~");
    ros::Publisher coeff_pub;
    ros::Subscriber sub;
    ros::Subscriber odom_sub;

    int grade=4;    // grade of the polynomial fitting
    int window=100;  // number of sample used in the least suqare trajectory fitting
                    // tuning this two params is fundamental to obtain a good prediction
    int counter=0;
    double offset;
    double t_off;
    double distance;
    std::deque<double> x_v, y_v, z_v, d_v, t_v;
    Eigen::VectorXd x;
    Eigen::VectorXd y;
    Eigen::VectorXd z;
    Eigen::VectorXd d;
    Eigen::VectorXd t;
    Eigen::VectorXd dronePos;
    Eigen::VectorXd drone_target;
    Eigen::VectorXd coeff_x;
    Eigen::VectorXd coeff_y;
    Eigen::VectorXd coeff_z;
    Eigen::VectorXd var;
    Eigen::MatrixXd W;
    Eigen::MatrixXd A;
    // Eigen::MatrixXd A_der = Eigen::MatrixXd(window,grade);
    trajectory_fitting::Coefficients coeff;

};

TrajectoryFitting::TrajectoryFitting() {
    // Subscribe to a given topic

    if(node.getParam("polynomial_grade", grade))
        ROS_INFO("Found parameter: setting polynomial grade to %d", grade);
    else    {
        ROS_INFO("Can't find polynomial grade");
    }
    if(node.getParam("window_dimension", window))
        ROS_INFO("Found parameter: setting interpolation window to %d", window);
    else    {
        ROS_INFO("Can't find interpolation window dimension");
    }


    x = Eigen::VectorXd(window);
    y = Eigen::VectorXd(window);
    z = Eigen::VectorXd(window);
    d = Eigen::VectorXd(window);
    t = Eigen::VectorXd(window);
    dronePos = Eigen::VectorXd(3);
    drone_target = Eigen::VectorXd(3);
    coeff_x = Eigen::VectorXd(grade);
    coeff_y = Eigen::VectorXd(grade);
    coeff_z = Eigen::VectorXd(grade);
    var = Eigen::VectorXd(window);
    W = Eigen::MatrixXd(window,window);
    A = Eigen::MatrixXd(window,grade);

    dronePos << 0,0,0;


    offset = ros::Time::now().toSec();
    sub = node.subscribe("/target_pos", 1, &TrajectoryFitting::callback, this);
    odom_sub = node.subscribe("/mavros/local_position/odom", 1, &TrajectoryFitting::OdomCallback, this);
    coeff_pub = node.advertise<trajectory_fitting::Coefficients>("/trajectory_fitting/coefficients", 1);
}


void TrajectoryFitting::OdomCallback(const nav_msgs::Odometry::ConstPtr& odometry){

    dronePos(0) = odometry->pose.pose.position.x;
    dronePos(1) = odometry->pose.pose.position.y;
    dronePos(2) = odometry->pose.pose.position.z;

}

void TrajectoryFitting::callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    ROS_INFO("Allah is well");
    t_off = msg->header.stamp.toSec() - offset;
    // insert last sample at the beginning of the vector
    // std::cout << "Entered the callback"<< std::endl;
    // TODO fix reference system between gazebo and SITL

    x_v.push_front(- msg->point.y);
    y_v.push_front(msg->point.x);
    z_v.push_front(msg->point.z);
    t_v.push_front(t_off);

    // std::cout << "assigned target pos"<< std::endl;

    drone_target(0) = -y_v[1] - dronePos(0);
    drone_target(1) = x_v[1] - dronePos(1);
    drone_target(2) = z_v[1] - dronePos(2);
    distance = drone_target.norm();

    // std::cout << "distance = " << distance << std::endl;
    // std::cout << "drone_target = " << std::endl << drone_target.transpose() << std::endl;

    d_v.push_front(distance);

    // std::cout << "Entered the callback" << " " <<d_v.size() << "/25" << std::endl;
    if(x_v.size() >= window) { // with less than #window samples risk to overfit
        // Remove the older sample
        x_v.pop_back();
        y_v.pop_back();
        z_v.pop_back();
        t_v.pop_back();
        d_v.pop_back();
        if(t_off < 10){
            for(int i=0 ; i<window ; i++ )  {
                x(i) = x_v[i];
                y(i) = y_v[i];
                z(i) = z_v[i];
                d(i) = d_v[i];
                t(i) = t_v[i];
            }
        }
        else {
            offset =  ros::Time::now().toNSec()*pow(10,-9)-0.1025;
            double m = t_v[1];
                for(int i=0 ; i<window ; i++ )  {
                    t_v[i] = t_v[i] - m ;
                    x(i) = x_v[i];
                    y(i) = y_v[i];
                    z(i) = z_v[i];
                    d(i) = d_v[i];
                    t(i) = t_v[i];
                }
        }
        // std::cout << "t = " << std::endl << t.transpose() << std::endl;


        //populate LS matrix with time powers for polynomial fitting
        for(int i=0; i<grade; i++) {
            double a = 1.0*i;
            A.col(i) = t.array().pow(a);
        }

        // std::cout << "A = " << std::endl << A << std::endl;


        //TODO : METTERE INSIEME LE DUE OPERAZIONI SOTTO
        var = d.array().square().pow(-1);
        W = var.matrix().asDiagonal();

        // suppose variance proportional to square dist., weight prop. to inverse variance

        // Resolve the LS problem with qr decomp.

        coeff_x = (A.transpose()*W*A).inverse()*A.transpose()*W*x;
        coeff_y = (A.transpose()*W*A).inverse()*A.transpose()*W*y;
        coeff_z = (A.transpose()*W*A).inverse()*A.transpose()*W*z;

        for(int i=0; i<grade; i++) {
            // TODO: there should be a better way to map
            coeff.coeffX.push_back(coeff_x(i));
            coeff.coeffY.push_back(coeff_y(i));
            coeff.coeffZ.push_back(coeff_z(i));
        }
        coeff.offset = offset;
        coeff_pub.publish(coeff);
        coeff.coeffX.clear();
        coeff.coeffY.clear();
        coeff.coeffZ.clear();
    }   // end of if on dimension;
} //end of the callback

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_fitter");
    TrajectoryFitting trajectory_fitting;

    // Enter a loop, pumping callbacks
    // ros::spin();
    ros::Rate loop_rate(10);

    while ( ros::ok() )
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("Closing...");
    return 0;
}
