#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense> // Include Eigen library for matrix operations
//for sin()
#include <math.h>
#include<array>

#include <cmath>
#include <tuple>

// Function to calculate inverse kinematics
float* IK(float x, float y, float theta, float l1, float l2, float l3) {
    static float result[5];
    // Calculate wrist position
    float xw = x - l3 * cos(theta*M_PI/180);
    float yw = y - l3 * sin(theta*M_PI/180);

    // Calculate elbow joint angle (theta2)
    float cos_theta2 = (xw * xw + yw * yw - l1 * l1 - l2 * l2) / (2 * l1 * l2);
    float theta2 = acos(cos_theta2);

    // Calculate shoulder joint angle (theta1)
    float theta1 = atan2(yw, xw) - atan2(l2 * sin(theta2), l1 + l2 * cos(theta2));

    // Calculate wrist joint angle (theta3)
    float theta3 = theta - theta1 - theta2;

    // Convert angles to degrees

    result[0] = theta1;
    result[1] = 0;
    result[2] = theta2;
    result[3] = 0;
    result[4] = theta3;

    return result;
}


// Function to calculate fifth-order polynomial coefficients
struct PolynomialResult {
    std::vector<double> coefficients;
    std::vector<double> time;
    std::vector<double> theta;
    std::vector<double> theta_dot;
    std::vector<double> theta_dotdot;
};

PolynomialResult calculateFifthOrderPolynomial(double t_f, int timesteps_per_second,
                                               double theta_0, double theta_f,
                                               double theta_dot_0, double theta_dot_f,
                                               double theta_dotdot_0, double theta_dotdot_f) {
    // Create the time array
    int num_steps = static_cast<int>(t_f * timesteps_per_second);
    std::vector<double> time(num_steps);
    for (int i = 0; i < num_steps; ++i) {
        time[i] = i * (t_f / num_steps);
    }

    // Coefficients for the matrix
    double a1 = std::pow(t_f, 3);
    double b1 = std::pow(t_f, 4);
    double c1 = std::pow(t_f, 5);
    double a2 = 3 * std::pow(t_f, 2);
    double b2 = 4 * std::pow(t_f, 3);
    double c2 = 5 * std::pow(t_f, 4);
    double a3 = 6 * t_f;
    double b3 = 12 * std::pow(t_f, 2);
    double c3 = 20 * std::pow(t_f, 3);

    // Constants matrix
    double d1 = theta_f - theta_0 - theta_dot_0 * t_f - 0.5 * theta_dotdot_0 * std::pow(t_f, 2);
    double d2 = theta_dot_f - theta_dot_0 - theta_dotdot_0 * t_f;
    double d3 = theta_dotdot_f - theta_dotdot_0;

    // Matrix A and B using Eigen library
    Eigen::Matrix3d A;
    A << a1, b1, c1,
         a2, b2, c2,
         a3, b3, c3;

    Eigen::Vector3d B(d1, d2, d3);

    // Solve for a_3, a_4, a_5
    Eigen::Vector3d solution = A.colPivHouseholderQr().solve(B);

    double a_0 = theta_0;
    double a_1 = theta_dot_0;
    double a_2 = theta_dotdot_0 / 2;  // Divided by 2 because it's part of the quadratic term
    double a_3 = solution(0);
    double a_4 = solution(1);
    double a_5 = solution(2);

    // Calculate theta, theta_dot, theta_dotdot over time
    std::vector<double> theta(num_steps), theta_dot(num_steps), theta_dotdot(num_steps);

    for (int i = 0; i < num_steps; ++i) {
        double t = time[i];
        theta[i] = a_0 + a_1 * t + a_2 * t * t + a_3 * std::pow(t, 3) + a_4 * std::pow(t, 4) + a_5 * std::pow(t, 5);
        theta_dot[i] = a_1 + 2 * a_2 * t + 3 * a_3 * std::pow(t, 2) + 4 * a_4 * std::pow(t, 3) + 5 * a_5 * std::pow(t, 4);
        theta_dotdot[i] = 2 * a_2 + 6 * a_3 * t + 12 * a_4 * std::pow(t, 2) + 20 * a_5 * std::pow(t, 3);
    }

    // Return the results
    return {
        {a_0, a_1, a_2, a_3, a_4, a_5},
        time,
        theta,
        theta_dot,
        theta_dotdot
    };
}

void publish(std_msgs::Float64 angles[5], ros::Publisher& pub1, ros::Publisher& pub2, ros::Publisher& pub3, ros::Publisher& pub4, ros::Publisher& pub5){
    pub1.publish(angles[0]);
    pub2.publish(angles[1]);
    pub3.publish(angles[2]);
    pub4.publish(angles[3]);
    pub5.publish(angles[4]);
    ROS_INFO("Publishing angle values testttttt: %f, %f, %f, %f, %f", angles[0].data, angles[1].data, angles[2].data, angles[3].data, angles[4].data);
}


int main(int argc, char **argv) {

    float l1 = 14.2;
    float l2 = 14.2;
    float l3 = 4.5;
    float x = l1+l2+l3-0.05;
    float y = 0;
    float theta = 0;

    std_msgs::Float64 initial_angles_to_publish[5];
    std_msgs::Float64 angles_to_publish[5];
    angles_to_publish[0].data = 0;
    angles_to_publish[1].data = 0;
    angles_to_publish[2].data = 0;
    angles_to_publish[3].data = 0;
    angles_to_publish[4].data = 0;
    initial_angles_to_publish[0].data = 0;
    initial_angles_to_publish[1].data = 0;
    initial_angles_to_publish[2].data = 0;
    initial_angles_to_publish[3].data = 0;
    initial_angles_to_publish[4].data = 0;

    ros::init(argc, argv, "rotate");

    ros::NodeHandle nh;

    ros::Publisher pub2 = nh.advertise<std_msgs::Float64>("/joint2/command", 100);
    ros::Publisher pub3 = nh.advertise<std_msgs::Float64>("/joint4/command", 100);
    ros::Publisher pub4 = nh.advertise<std_msgs::Float64>("/joint6/command", 100);
    ros::Publisher pub5 = nh.advertise<std_msgs::Float64>("/end/command", 100);
    ros::Publisher pub1 = nh.advertise<std_msgs::Float64>("/motortom2m/command", 100);

    ros::Rate loop_rate(50);
    
    float* IK_results;
    IK_results = IK(x, y, theta, l1, l2, l3);
    for(int i = 0; i<5; i++){
            initial_angles_to_publish[i].data = IK_results[i];
    }
    
    ros::Time startTime = ros::Time::now();

    x = 20;
    y = 15;
    theta = 0;
    double t_f = 10;
    int timesteps_per_second = 50;
    double theta_0 = 0;
    IK_results = IK(x, y, theta, l1, l2, l3);
    int count = 0;
    int count1 = 0;

    PolynomialResult result1 = calculateFifthOrderPolynomial(t_f, timesteps_per_second, theta_0, IK_results[0], 0, 0, 0, 0);
    PolynomialResult result2 = calculateFifthOrderPolynomial(t_f, timesteps_per_second, theta_0, IK_results[2], 0, 0, 0, 0);
    PolynomialResult result3 = calculateFifthOrderPolynomial(t_f, timesteps_per_second, theta_0, IK_results[4], 0, 0, 0, 0);

    while (ros::ok()) {
        // Calculate elapsed time
        ros::Duration elapsed = ros::Time::now() - startTime;
        double time = elapsed.toSec();

        if(time < 5){
            publish(initial_angles_to_publish, pub1, pub2, pub3, pub4, pub5);
            ROS_INFO("Publishing initial values");
        }

        if(time > 5){
            angles_to_publish[0].data = result1.theta[count];
            angles_to_publish[1].data = 0;
            angles_to_publish[2].data = result2.theta[count];
            angles_to_publish[3].data = 0;
            angles_to_publish[4].data = result3.theta[count++];
            publish(angles_to_publish, pub1, pub2, pub3, pub4, pub5);
            ROS_INFO("Publishing trajectory values");
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}