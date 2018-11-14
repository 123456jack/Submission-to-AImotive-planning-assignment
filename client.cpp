/**
 * My Solution to the AImotive planning assignement 
 *
 * Copyright 2018 by Ziad ZAMZAMI <zamzami@isir.upmc.fr>
 *
 * This file is part of my Solution to the AImotive planning assignement.
 * 
 * You can redistribute it and/or modify it under the terms of the GNU General Public 
 * License as published by the Free Software Foundation, either version 3 of the License.
 * 
 * It is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
 * See theGNU General Public License for more details.
 *
 * @license GPL-3.0+ <http://spdx.org/licenses/GPL-3.0+>
 */

#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <ignition/math/Rand.hh>

#include "custom_messages.pb.h"
#include "aimotive_mpc.h"

#include <gazebo/gazebo_client.hh>

/**
 * Minimal client application to connect to the simulator, receive the
 * current world state and send the next ego vehicle velocity.
 * You need to extend the OnWorldStateReceived function, the rest
 * is the boilerplate code for communication.
 */

typedef const boost::shared_ptr<
        const custom_messages::WorldState>
        WorldStateRequestPtr;

typedef const boost::shared_ptr<
        const custom_messages::Statistics>
        StatisticsRequestPtr;

//Define pi constant.
constexpr double pi() {return M_PI;}
// Define variables to store the ego vehicle acceleration and jerk.
double ego_car_acc = 0;
double ego_car_jerk = 0;

class Controller {
    private: gazebo::transport::NodePtr node;

    private: gazebo::transport::SubscriberPtr world_sub;
    private: gazebo::transport::SubscriberPtr statistics_sub;
    private: gazebo::transport::PublisherPtr pub;

    private: std::string worldTopicName = "~/world_state";
    private: std::string statisticsTopicName = "~/statistics";
    private: std::string commandTopicName = "~/client_command";

    private: int32_t simulation_round = 0;

    private: bool Collision_predicted= false;

    public: void Init()
    {
        // Create our node for communication
        node = gazebo::transport::NodePtr(new gazebo::transport::Node());
        node->Init();

        // Subscribe to the topic, and register a callback
        this->world_sub = node->Subscribe(worldTopicName, &Controller::OnWorldStateReceived, this);

        this->statistics_sub = node->Subscribe(statisticsTopicName, &Controller::OnStatisticsReceived, this);

        // Publish to the velodyne topic
        this->pub = node->Advertise<custom_messages::Command>(commandTopicName);

        // Wait for a subscriber to connect to this publisher
        this->pub->WaitForConnection();
    }

    // Called when the simulator resets the world after reaching the goal, running out of time or crashing
    public: void ResetWorld()
    {
        std::cout << "New simulation round started, resetting world." << std::endl;
    }

    public: void PrintWorldStateMessage(WorldStateRequestPtr& msg) const
    {
        std::cout << "Simulation round: " << msg->simulation_round() << "; "
                  << "time: " << msg->time().sec() << "." << msg->time().nsec() << std::endl;
        std::cout << "  ego_car p: (" << msg->ego_vehicle().position().x() << ", " << msg->ego_vehicle().position().y()
                  << ") v: (" << msg->ego_vehicle().velocity().x() << ", " << msg->ego_vehicle().velocity().y() << "); " << std::endl;
        for (const auto& vehicle_msg : msg->vehicles())
        {
            std::cout << "  car id " << vehicle_msg.vehicle_id()
                      << " lane id: " << vehicle_msg.lane_id()
                      << " p: (" << vehicle_msg.position().x() << ", " << vehicle_msg.position().y()
                      << ") v: (" << vehicle_msg.velocity().x() << ", " << vehicle_msg.velocity().y() << "); "
                      << std::endl;
        }

        std::cout << std::endl;
    }

    public: void PrintStatisticsMessage(StatisticsRequestPtr& msg) const
    {
        std::cout << "Statistics from previous round: "
                  << "Success: " << msg->success() << ", "
                  << "collision: " << msg->collision_detected() << ", "
                  << "Time steps: "
                  << msg->simulation_time_steps_taken() << ", "
                  << "total acceleration: " << msg->total_acceleration() << ", "
                  << "acceleration/jerk limits respected: " << msg->limits_respected()
                  << std::endl;
    }

    // Called every time a new update is received from the simulator.
    // This is the main function where you should implement your code
    public: void OnWorldStateReceived(WorldStateRequestPtr& msg)
    {
        PrintWorldStateMessage(msg);

        // If the simulation round is different then this is a whole new setting, reinitialize the world
        if (msg->simulation_round() != simulation_round)
        {
            ResetWorld();
            simulation_round = msg->simulation_round();
        }

       // Collision prediction algorithm based on the collision cone concept.
        //
        // Simple variables to store the position and velocity of the ego vehicle.
        double ego_car_position = msg->ego_vehicle().position().x();
        double ego_car_velocity = msg->ego_vehicle().velocity().x();

        // Find upcoming vehicles from the priority lane.
        for (const auto& vehicle_msg : msg->vehicles()) {
            if (vehicle_msg.lane_id() == 1) {
                 
                 // store contestant vehicle position and velocity.
                 double contestant_car_position = vehicle_msg.position().y();
                 double contestant_car_velocity = vehicle_msg.velocity().y();

                // The radius (center to front) of each vehicle can be found from the collsion geometry.
                 const double eg_vehicle_radius = 2.5;
                 const double contestant_vehicle_radius = 2.25;
                 const double conservative_margin = 0.25;
                
                // Represent the contestant vehicle as a configuration space obstacle
                // for the ego vehicle with a radius R. The configuration space for the ego vehicle 
                // is constructed by reducing its geometry to a point and augmenting the obstacle's geometry by its radius.
                 const double R = contestant_vehicle_radius + eg_vehicle_radius + conservative_margin;
                 
                 // Collision Prediction based on the kinematics of the Line Of Sight (LOS) and the collision cone.
                 // Please refer to  "Chakravarthy, A., & Ghose, D. (1998). Obstacle avoidance in a dynamic environment: A collision cone approach. 
                 // IEEE Transactions on Systems, Man, and Cybernetics-Part A: Systems and Humans, 28(5), 562-574".
                 double LOS_r = 0;
                 double LOS_theta = 0;
                 double const alpha = 0;
                 double const beta = pi()/2; 
                 double Vr = 0;
                 double Vtheta = 0;
                 double p = 0;
                 LOS_r= sqrt(((2 - ego_car_position)* (2- ego_car_position)) + ((contestant_car_position + 2)*(contestant_car_position + 2))); 
                 LOS_theta= pi()- atan(contestant_car_position/ego_car_position);  
                 // Eq(1) in the referenced paper.
                 Vr= contestant_car_velocity* cos(beta-LOS_theta)- ego_car_velocity* cos(0-LOS_theta);
                 // Eq(2) in the referenced paper.
                 Vtheta=contestant_car_velocity* sin(beta-LOS_theta)- ego_car_velocity* sin(0-LOS_theta);
                 // Eq (25) in the referenced paper.
                 p = R/sqrt(pow(LOS_r,2)-pow(R,2));

                 // for debugging 
                // std::cout<<"LOS-r:"<<LOS_r<<"\n"; 
                // std::cout<<"LOS-theta: "<<LOS_theta<<"\n"; 
                // std::cout<<"Vr: "<<Vr<<"\n"; 
                // std::cout<<"Vtheta: "<<Vtheta<<"\n"; 
                //  std::cout<<"p: "<<p<<"\n";
                
                 double lhs=pow(Vtheta,2);
                 double rhs=pow(Vr,2)*pow(p,2);
                 // Eq(24) in the referenced paper.
                 if (lhs<rhs){ 
                     Collision_predicted= true;
                     //for debugging 
                    // std::cout<<"========= COLLISION PREDICTED================ \n";
                 };
               
               // for debugging
               /*  std::cout<<"========================= \n"; 
                 std::cout << "enemy   car id " << vehicle_msg.vehicle_id()
                      << " enemy lane id: " << vehicle_msg.lane_id()
                      << " enemy p: (" << vehicle_msg.position().x() << ", " << vehicle_msg.position().y()
                      << ") enemy v: (" << vehicle_msg.velocity().x() << ", " << vehicle_msg.velocity().y() << "); "
                      << std::endl;
                 std::cout<<"========================= \n"; */
             }
        }

        

        // Calculate the next velocity for the ego car and send the response.
        // Implement your code here
        custom_messages::Command response_msg;
        // response_msg.set_ego_car_speed(ignition::math::Rand::DblUniform(5, 15));
            // Define a reference velocity based on the collision prediction algorithm.
            double v_ref; 

            if (Collision_predicted) {
                v_ref = 0;
            } else {
                v_ref = 20;
            }
         // Construct MPC controller. 
         AimotiveMpc mpc;
        const auto result = mpc.Solve(ego_car_position, ego_car_velocity, ego_car_acc, ego_car_jerk, v_ref );
        // Update acceleration and jerk for next iteration 
        ego_car_acc = result[1] ;
        ego_car_jerk = result[2] ;
        double  mpc_vel = result[0] ;
        double  mpc_acc = result[1] ;
        double  mpc_jerk = result[2] ;
        double dt = 0.1; 

       // For debgging only.
       // double  next_vel = ego_car_velocity + mpc_acc * dt + 0.5* pow(dt, 2) * mpc_jerk  ;
       // std::cout << "Current velocity   :"<<ego_car_velocity<<" ========================= \n";
       // std::cout << "MPC velocity result:"<<mpc_vel<<" ========================= \n";
       // std::cout << "next velocity result:"<<next_vel<<" ========================= \n";
       // std::cout << "MPC acc result:"<<mpc_acc<<" ========================= \n";
       // std::cout << "MPC jerk result:"<<mpc_jerk<<" ========================= \n";

        response_msg.set_ego_car_speed(mpc_vel);
        //response_msg.set_ego_car_speed(20); 
        response_msg.set_simulation_round(msg->simulation_round());
        this->pub->Publish(response_msg);
    }

    public: void OnStatisticsReceived(StatisticsRequestPtr& msg)
    {
        PrintStatisticsMessage(msg);
    }
};

int main(int _argc, char **_argv)
{
    // Load gazebo as a client
    gazebo::client::setup(_argc, _argv);

    Controller controller;
    controller.Init();


     while (true)
     gazebo::common::Time::MSleep(100);

    // Make sure to shut everything down.
    gazebo::client::shutdown();
}
