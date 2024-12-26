#include <iostream>
#include <ros/ros.h>
#include <cbnu_msgs/AllObjectsData.h>
#include <cbnu_msgs/VehicleTlm.h>
#include <cbnu_msgs/VehicleCmd.h>
#include <std_msgs/Float64.h>
#include <cmath>
#include <algorithm>
#include <vector>
#include <limits>
#include <matplotlibcpp.h>
#include <pybind11/embed.h>

namespace plt = matplotlibcpp;

class IDMCalculator
{
    double obj_x, obj_y; //선행차량 위치 변수
    double fvel_x, fvel_y, fvel_z; // 선행차량 
    double fvel; //선행차량 속력
    double current_vel, current_accel; // ego vehicle 속도, 가속도 
    double distance; // 선행차량과의 거리 
    double delta_v; // ego vehicle - front vehicle
    double v_desired; // 목표 속도
    double prev_filtered_accel;
    bool veh_info_received;
    bool obj_info_received;
    
    ros::Publisher desired_v_publisher;
    ros::Publisher accel_publisher;
    ros::Publisher vel_publisher;

    struct LookupTableEntry{
        double fvel;
        double s0_adjusted;
        double T_adjusted;
        double v0_adjusted;
    };

    std::vector<LookupTableEntry> lookup_table = {
        {42.0, 5.0, 1.0, 35.0}, //151.2
        {25.0, 5.0, 1.0, 30.0}, //90.0
        {22.0, 5.0, 1.0, 25.0}, //79.2
        {16.0, 5.0, 1.0, 18.0}, //57.6
        {12.0, 5.0, 1.0, 15.0}, //43.2
        {8.0, 5.0, 1.0, 10.0}, //28.8
        {5.0, 5.0, 1.0, 13.0}, //18.0
        {2.3, 5.0, 1.0, 5.0} //8.28
    };

    void interpolate(double fvel, double& s0_adjusted, double& T_adjusted, double& v0_adjusted);

public:
    IDMCalculator(ros::NodeHandle& nh) : obj_x(0), obj_y(0), fvel(0), current_vel(0), current_accel(0),
                      distance(0), delta_v(0), v_desired(0), veh_info_received(false), obj_info_received(false){

        desired_v_publisher = nh.advertise<cbnu_msgs::VehicleCmd>("/ctrl_cmd", 1000);
        accel_publisher = nh.advertise<std_msgs::Float64>("/calculated_acceleration", 1000);
        vel_publisher = nh.advertise<std_msgs::Float64>("/desired_vel",1000);
    }
    void ObjectDataCallback(const cbnu_msgs::AllObjectsData::ConstPtr& obj);
    void VehicleInfoCallback(const cbnu_msgs::VehicleTlm::ConstPtr& vehicle_info);
    double calAcceleration(double current_vel, double delta_v, double distance, double s0, double T, double v0, double a = 2.5, double b = 5, double delta = 4.0);
    // 값 조정 (Spacing space headway, headway time, max acceleration, max deceleration, max velocity, acceleration exponent)
    void operate();
    void plotData();
    void publishDesiredVelocity();
};

void IDMCalculator::interpolate(double fvel, double& s0_adjusted, double& T_adjusted, double& v0_adjusted)
{
    if (fvel >= lookup_table.front().fvel)
    {
        s0_adjusted = lookup_table.front().s0_adjusted;
        T_adjusted = lookup_table.front().T_adjusted;
        v0_adjusted = lookup_table.front().v0_adjusted;
        return;
    }
    else if (fvel <= lookup_table.back().fvel) {
        s0_adjusted = lookup_table.back().s0_adjusted;
        T_adjusted = lookup_table.back().T_adjusted;
        v0_adjusted = lookup_table.back().v0_adjusted;
        return;
    }
    for (size_t i = 0; i < lookup_table.size() - 1; ++i) {
        if (fvel < lookup_table[i].fvel && fvel > lookup_table[i + 1].fvel) {
            double x1 = lookup_table[i].fvel;
            double x2 = lookup_table[i + 1].fvel;

            // 선형 보간 공식: y = y1 + (x - x1) * (y2 - y1) / (x2 - x1)
            s0_adjusted = lookup_table[i].s0_adjusted + (fvel - x1) * (lookup_table[i + 1].s0_adjusted - lookup_table[i].s0_adjusted) / (x2 - x1);
            T_adjusted = lookup_table[i].T_adjusted + (fvel - x1) * (lookup_table[i + 1].T_adjusted - lookup_table[i].T_adjusted) / (x2 - x1);
            v0_adjusted = lookup_table[i].v0_adjusted + (fvel - x1) * (lookup_table[i + 1].v0_adjusted - lookup_table[i].v0_adjusted) / (x2 - x1);
            return;
        }
    }
}




void IDMCalculator::VehicleInfoCallback(const cbnu_msgs::VehicleTlm::ConstPtr& vehicle_info)
{
    current_vel = vehicle_info->speed;
    current_accel = vehicle_info->acceleration_forward;
    veh_info_received = true;

    //ROS_INFO("Vehicle info received: speed = %f, acceleration = %f", current_vel, current_accel);
}


void IDMCalculator::ObjectDataCallback(const cbnu_msgs::AllObjectsData::ConstPtr& obj)
{
    static double previous_fvel = 0.0;

    for (const auto& obj_data : obj->object_data_array)
    {
        if (obj_data.object_type == "SurroundVehicle")
        {
            obj_x = obj_data.rel_pos_x;
            obj_y = obj_data.rel_pos_y;
            distance = std::sqrt(std::pow(obj_x, 2) + std::pow(obj_y, 2));
            //ROS_INFO("distance: %f", distance);
            fvel = obj_data.global_velocity / 3.6;
            delta_v = current_vel - fvel; 


            obj_info_received = true;

            //ROS_INFO("Object info received: distance = %f, fvel = %f", distance, fvel);
        }
    }
    operate();
    //ROS_INFO("operate");
}



double IDMCalculator::calAcceleration(double current_vel, double delta_v, double distance, double s0, double T, double v0, double a, double b, double delta)
{
   double s_star = s0 + current_vel * T + (current_vel * delta_v) / (2 * sqrt(a * b));
   double acceleration = a * (1 - pow(current_vel / v0, delta) - pow(s_star / distance, 2));
   //ROS_INFO("s_star: %f", s_star);
   return acceleration; // 가속도 반환
}

//kph, mps

void IDMCalculator::operate()
{
    if (veh_info_received && obj_info_received)
    {
        double dt = 0.02;
        if (dt > 0)
        {
            double acceleration = 0.0;
            double s0_adjusted = 0.0;
            double T_adjusted = 0.0;
            double v0_adjusted = 0.0;

            if(std::abs(fvel) < 1.0 && distance <= 50.0)
            {
                v_desired = 0.0;
            }
            else  // 이 부분 튜닝
            {   
                interpolate(fvel, s0_adjusted, T_adjusted, v0_adjusted);
                
                acceleration = calAcceleration(current_vel, delta_v, distance, s0_adjusted, T_adjusted, v0_adjusted);
                acceleration = std::max(-5.0, std::min(acceleration, 2.0));
                ROS_INFO("calAcc: %f", acceleration);

                v_desired = v_desired + acceleration * dt; 
                
            }

            std_msgs::Float64 accel_msg;
            accel_msg.data = acceleration;
            accel_publisher.publish(accel_msg);

            std_msgs::Float64 vel_msg;
            vel_msg.data = v_desired;
            vel_publisher.publish(vel_msg);


            publishDesiredVelocity();
        }
        veh_info_received = false;
        obj_info_received = false;
    }
    

}



void IDMCalculator::publishDesiredVelocity()
{
    cbnu_msgs::VehicleCmd msg;

    if (v_desired < 0.0) {
        v_desired = 0.0;
    }

    msg.velocity = v_desired * 3.6;
   // msg.acceleration = calAcceleration();
    msg.longlCmdType = 2;
    msg.gear_cmd = 4;
    desired_v_publisher.publish(msg);
}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "acc_node");
    ros::NodeHandle n;

    IDMCalculator idm_cal(n);

    ros::Subscriber sub_vehicle = n.subscribe("/Ego_topic", 1000, &IDMCalculator::VehicleInfoCallback, &idm_cal);
    ros::Subscriber sub_object = n.subscribe("/Object_topic", 1000, &IDMCalculator::ObjectDataCallback, &idm_cal);

    ros::Rate loop_rate(50);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

// #include <iostream>
// #include <ros/ros.h>
// #include <cbnu_msgs/AllObjectsData.h>
// #include <cbnu_msgs/VehicleTlm.h>
// #include <cbnu_msgs/VehicleCmd.h>
// #include <std_msgs/Float64.h>
// #include <cmath>
// #include <algorithm>
// #include <vector>
// #include <limits>
// #include <matplotlibcpp.h>
// #include <pybind11/embed.h>


// namespace plt = matplotlibcpp;

// class IDMCalculator
// {
//     double obj_x, obj_y; //선행차량 위치 변수
//     double fvel_x, fvel_y, fvel_z; // 선행차량 
//     double fvel; //선행차량 속력
//     double current_vel, current_accel; // ego vehicle 속도, 가속도
//     double distance; // 선행차량과의 거리
//     double delta_v; // ego vehicle - front vehicle
//     double v_desired; // 목표 속도
//     double alpha; // LPF
//     double prev_filtered_accel;
//     bool veh_info_received;
//     bool obj_info_received;
//     ros::Time info_prev_time;
//     ros::Time ttc_prev_time;
//     ros::Time lead_pvel_time;

//     std::vector<double> time_data;
//     std::vector<double> front_vel_data;
//     std::vector<double> front_veltime_data;
//     std::vector<double> front_accel_data;
//     std::vector<double> ttc_time_data;
//     std::vector<double> distance_data;
//     std::vector<double> current_vel_data;
//     std::vector<double> time_to_collision_data;
//     std::vector<double> acceleration_data;
//     std::vector<double> accel_time_data;
//     ros::Publisher desired_v_publisher;
//     ros::Publisher accel_publisher;
//     ros::Publisher vel_publisher;

//     struct LookupTableEntry{
//         double fvel;
//         double s0_adjusted;
//         double T_adjusted;
//         double v0_adjusted;
//     };

//     std::vector<LookupTableEntry> lookup_table = {
//         {42.0, 5.0, 1.2, 35.0},
//         {25.0, 5.0, 1.0, 30.0},
//         {22.0, 5.0, 1.0, 25.0},
//         {16.0, 5.0, 1.3, 18.0},
//         {12.0, 5.0, 1.3, 15.0},
//         {8.0, 15.0, 1.3, 10.0},
//         {5.0, 20.0, 1.5, 13.0},
//         {2.3, 35.0, 1.5, 5.0}
//     };

//     void interpolate(double fvel, double& s0_adjusted, double& T_adjusted, double& v0_adjusted);

// public:
//     IDMCalculator(ros::NodeHandle& nh) : obj_x(0), obj_y(0), fvel(0), fvel_y(0), fvel_z(0), current_vel(0), current_accel(0),
//                       distance(0), delta_v(0), v_desired(0), veh_info_received(false), obj_info_received(false), alpha(0.01), prev_filtered_accel(0.0){
//         info_prev_time = ros::Time::now();
//         ttc_prev_time = ros::Time::now();
//         lead_pvel_time = ros::Time::now();
//         desired_v_publisher = nh.advertise<cbnu_msgs::VehicleCmd>("/ctrl_cmd", 1000);
//         accel_publisher = nh.advertise<std_msgs::Float64>("/calculated_acceleration", 1000);
//         vel_publisher = nh.advertise<std_msgs::Float64>("/desired_vel",1000);
//     }
//     void ObjectDataCallback(const cbnu_msgs::AllObjectsData::ConstPtr& obj);
//     void VehicleInfoCallback(const cbnu_msgs::VehicleTlm::ConstPtr& vehicle_info);
//     double calAcceleration(double current_vel, double delta_v, double distance, double s0, double T, double v0, double a = 2.5, double b = 5, double delta = 4.0);
//     // 값 조정 (Spacing space headway, headway time, max acceleration, max deceleration, max velocity, acceleration exponent)
//     void operate();
//     void plotData();
//     void publishDesiredVelocity();
//     void applyLowPassFilter(std::vector<double>& data, double alpha);
// };

// void IDMCalculator::interpolate(double fvel, double& s0_adjusted, double& T_adjusted, double& v0_adjusted)
// {

// }

// void IDMCalculator::applyLowPassFilter(std::vector<double>& data, double alpha)
// {
//     if (data.empty()) return;

//     std::vector<double> filtered_data;
//     filtered_data.push_back(data[0]); 

//     for (size_t i = 1; i < data.size(); ++i)
//     {
//         double filtered_value = alpha * data[i] + (1.0 - alpha) * filtered_data[i - 1];
//         filtered_data.push_back(filtered_value);
//     }

//     data = filtered_data;
// }



// void IDMCalculator::VehicleInfoCallback(const cbnu_msgs::VehicleTlm::ConstPtr& vehicle_info)
// {
//     current_vel = vehicle_info->speed;
//     ros::Time lead_cvel_time = ros::Time::now();
//     current_accel = vehicle_info->acceleration_forward;
//     veh_info_received = true;
//     info_prev_time = lead_cvel_time;
// }


// void IDMCalculator::ObjectDataCallback(const cbnu_msgs::AllObjectsData::ConstPtr& obj)
// {
//     static double previous_fvel = 0.0;

//     for (const auto& obj_data : obj->object_data_array)
//     {
//         if (obj_data.object_type == "Pedestrian")
//         {
//             obj_x = obj_data.rel_pos_x;
//             obj_y = obj_data.rel_pos_y;
//             distance = std::sqrt(std::pow(obj_x, 2) + std::pow(obj_y, 2));
//             fvel = obj_data.global_velocity / 3.6;
//             delta_v = current_vel - fvel; 

//             //선행차량 그래프
//             double lead_accel = (fvel - previous_fvel) / (ros::Time::now() - lead_pvel_time).toSec();

//             if (lead_accel >= -5.0 && lead_accel <= 5.0)
//             {
//                 front_accel_data.push_back(lead_accel);
//             }
//             else
//             {
//                 front_accel_data.push_back(0.0);
//             }

//             previous_fvel = fvel;
//             lead_pvel_time = ros::Time::now();

//             front_vel_data.push_back(fvel);
//             front_veltime_data.push_back(lead_pvel_time.toSec());

//             obj_info_received = true;
//         }
//     }
//     operate();
// }



// double IDMCalculator::calAcceleration(double current_vel, double delta_v, double distance, double s0, double T, double v0, double a, double b, double delta)
// {
//    double s_star = s0 + current_vel * T + (current_vel * delta_v) / (2 * sqrt(a * b));
//    double acceleration = a * (1 - pow(current_vel / v0, delta) - pow(s_star / distance, 2));
//    //ROS_INFO("s_star: %f", s_star);
//    return acceleration; // 가속도 반환
// }

// //kph, mps

// void IDMCalculator::operate()
// {
//     if (veh_info_received && obj_info_received)
//     {
//         ros::Time info_curr_time = ros::Time::now();
//         double dt = 0.02;
//         if (dt > 0)
//         {
//             double time_now = info_curr_time.toSec();
//             double acceleration = 0.0;
//             double s0_adjusted = 10.0;
//             double T_adjusted = 1.0;
//             double v0_adjusted = 28.0;

//             if(std::abs(fvel) < 1.0 && distance <= 50.0)
//             {
//                 v_desired = 0.0;
//             }
//             else  // 이 부분 튜닝
//             {   
                
//                 if(fvel < 42.0 && fvel > 25.0) //90
//                 {
//                     s0_adjusted = 10.0; //10
//                     T_adjusted = 1.2;
//                     v0_adjusted = 35.0;
//                 }
//                 else if(fvel < 25.0 && fvel > 22.0) // 79.2
//                 {
//                     s0_adjusted = 10.0;
//                     T_adjusted = 1.3;
//                     v0_adjusted = 30.0;
//                 }
//                 else if(fvel < 22.0 && fvel > 16.0) // 57.6
//                 {
//                     s0_adjusted = 10.0;
//                     T_adjusted = 1.0;
//                     v0_adjusted = 25.0;
//                 }
//                 else if(fvel < 16.0 && fvel > 12.0) // 43.2
//                 {
//                     s0_adjusted = 10.0;
//                     T_adjusted = 1.3;
//                     v0_adjusted = 18.0;
//                 }
//                 else if(fvel < 12.0 && fvel > 8.0) // 28.8
//                 {
//                     s0_adjusted = 10.0; // 5.0 1.3 13.0
//                     T_adjusted = 1.3;
//                     v0_adjusted = 15.0;
//                 }
//                 else if(fvel < 8.0 && fvel > 5.0) // 18.0
//                 {
//                     s0_adjusted = 10.0; //15
//                     T_adjusted = 1.3;
//                     v0_adjusted = 10.0;
//                 }
//                 else if(fvel < 5.0 && fvel > 2.3) // 8.0
//                 {
//                     s0_adjusted = 10.0; //20
//                     T_adjusted = 1.5;
//                     v0_adjusted = 13.0;
//                 }
//                 else if(fvel < 2.3 && fvel > 0.0) 
//                 {
//                     s0_adjusted = 10.0; //25
//                     T_adjusted = 1.5;
//                     v0_adjusted = 5.0;
//                 }
                
//                 acceleration = calAcceleration(current_vel, delta_v, distance, s0_adjusted, T_adjusted, v0_adjusted);
//                 acceleration = std::max(-5.0, std::min(acceleration, 2.0));
//                 ROS_INFO("calAcc: %f", acceleration);

//                 v_desired = v_desired + acceleration * dt;
//                 v_desired = std::min(v_desired, 6.94);
//                 //ROS_INFO("Desired Vel: %f", v_desired * 3.6);
//                 info_prev_time = info_curr_time;
//             }

//             std_msgs::Float64 accel_msg;
//             accel_msg.data = acceleration;
//             accel_publisher.publish(accel_msg);

//             std_msgs::Float64 vel_msg;
//             vel_msg.data = v_desired;
//             vel_publisher.publish(vel_msg);

//             time_data.push_back(time_now);
//             distance_data.push_back(distance);
//             current_vel_data.push_back(current_vel);
//             acceleration_data.push_back(acceleration);
//             accel_time_data.push_back(time_now);

//             double ttc;
//             const double epsilon = 1e-6;

//             if (delta_v < epsilon) {
//                 ttc = distance / -delta_v;
//             } else {
//                 ttc = std::numeric_limits<double>::infinity();
//             }

//             ros::Time ttc_curr_time = ros::Time::now();
//             time_to_collision_data.push_back(ttc);
//             double ttc_time_now = ttc_curr_time.toSec();
//             ttc_time_data.push_back(ttc_time_now);

//             publishDesiredVelocity();
//         }
//         veh_info_received = false;
//         obj_info_received = false;
//     }
    

// }



// void IDMCalculator::publishDesiredVelocity()
// {
//     cbnu_msgs::VehicleCmd msg;

//     if (v_desired < 0.0) {
//         v_desired = 0.0;
//     }

//     msg.velocity = v_desired * 3.6;
//    // msg.acceleration = calAcceleration();
//     msg.longlCmdType = 2;
//     msg.gear_cmd = 4;
//     desired_v_publisher.publish(msg);
// }

// void IDMCalculator::plotData()
// {
//     plt::figure_size(5910,1970);

//     double min_ttc = std::numeric_limits<double>::infinity();
//     double min_ttc_time = 0.0;

//     for (size_t i = 0; i < time_to_collision_data.size(); ++i) {
//         if (time_to_collision_data[i] > 0 && time_to_collision_data[i] < min_ttc) {
//             min_ttc = time_to_collision_data[i];
//             min_ttc_time = ttc_time_data[i];
//         }
//     }

//     std::cout << "최저 Time to Collision: " << min_ttc << "초, 시간: " << min_ttc_time << "초" << std::endl;

//     // Time vs Current Velocity and Ego Vehicle Acceleration
//     plt::subplot(2, 2, 4);
//     if (time_data.size() == current_vel_data.size() && accel_time_data.size() == acceleration_data.size()) {
//         plt::named_plot("Ego Vehicle Velocity", time_data, current_vel_data, "b-");
//         //plt::named_plot("Ego Vehicle Acceleration", accel_time_data, acceleration_data, "g-");
//         plt::xlabel("Time (s)");
//         plt::ylabel("Velocity (m/s) & Acceleration (m/s²)");
//         plt::title("Ego Vehicle Velocity vs Time");
//         plt::legend(); 
//         plt::grid(true);
//     } else {
//         std::cerr << "Time and Current Velocity or Acceleration data size mismatch." << std::endl;
//     }

//     // Time vs Distance
//     plt::subplot(2, 2, 2);
//     if (time_data.size() == distance_data.size()) {
//         plt::plot(time_data, distance_data);
//         plt::xlabel("Time (s)");
//         plt::ylabel("Distance (m)");
//         plt::title("Distance vs Time");
//         plt::grid(true); 
//     } else {
//         std::cerr << "Time and Distance data size mismatch." << std::endl;
//     }

//     // // Time vs Time to Collision
//     // plt::subplot(2, 2, 1);
//     // if (ttc_time_data.size() == time_to_collision_data.size()) {
//     //     plt::plot(ttc_time_data, time_to_collision_data);
//     //     plt::xlabel("Time (s)");
//     //     plt::ylabel("Time to Collision (s)");
//     //     plt::title("Time to Collision vs Time");
//     //     plt::ylim(0, 10);
//     //     plt::grid(true);

//     //     plt::plot({min_ttc_time}, {min_ttc}, "ro");

//     //     std::string text = "Min: " + std::to_string(min_ttc) + " s";
//     //     plt::text(min_ttc_time, min_ttc + 0.5, text);
//     // } else {
//     //     std::cerr << "Time to Collision data size mismatch." << std::endl;
//     //}

//     // // Time vs Lead Vehicle Speed and Lead Vehicle Acceleration
//     // plt::subplot(2, 2, 3);
//     // if (front_veltime_data.size() == front_vel_data.size() && front_veltime_data.size() == front_accel_data.size()) {
//     //     std::vector<double> filtered_accel_data = front_accel_data;
//     //     applyLowPassFilter(filtered_accel_data, 0.03);
//     //     plt::named_plot("Lead Vehicle Speed", front_veltime_data, front_vel_data, "r-");
//     //     plt::named_plot("Lead Vehicle Acceleration", front_veltime_data, filtered_accel_data, "m-");
//     //     plt::xlabel("Time (s)");
//     //     plt::ylabel("Speed (m/s) & Acceleration (m/s²)");
//     //     plt::title("Lead Vehicle Speed & Acceleration vs Time");
//     //     plt::legend();
//     //     plt::grid(true);
//     // } else {
//     //     std::cerr << "Lead Vehicle Time and Speed or Acceleration data size mismatch." << std::endl;
//     //}


//     plt::show();
// }



// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "acc_node");
//     ros::NodeHandle n;

//     IDMCalculator idm_cal(n);

//     ros::Subscriber sub_vehicle = n.subscribe("/Ego_topic", 1000, &IDMCalculator::VehicleInfoCallback, &idm_cal);
//     ros::Subscriber sub_object = n.subscribe("/Object_topic", 1000, &IDMCalculator::ObjectDataCallback, &idm_cal);

//     ros::Rate loop_rate(50);

//     while (ros::ok())
//     {
//         ros::spinOnce();
//         loop_rate.sleep();
//     }

//     idm_cal.plotData();

//     return 0;
// }



