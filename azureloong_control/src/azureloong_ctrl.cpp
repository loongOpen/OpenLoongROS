/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024 Humanoid Robot (Shanghai) Co., Ltd, under Apache 2.0.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/
#include <ros/ros.h>
#include <ros/time.h>
#include <ros/package.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include "GLFW_callbacks.h"
#include "MJ_interface.h"
#include "PVT_ctrl.h"
#include "data_logger.h"
#include "data_bus.h"
#include "pino_kin_dyn.h"
#include "useful_math.h"
#include "wbc_priority.h"
#include "mpc.h"
#include "gait_scheduler.h"
#include "foot_placement.h"
#include "joystick_interpreter.h"
#include <string>
#include <iostream>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_broadcaster.h>

const std::vector<std::string> JointName={ "J_arm_l_01", "J_arm_l_02", "J_arm_l_03", "J_arm_l_04", "J_arm_l_05", "J_arm_l_06", "J_arm_l_07",
    "J_arm_r_01", "J_arm_r_02", "J_arm_r_03", "J_arm_r_04", "J_arm_r_05", "J_arm_r_06", "J_arm_r_07",
    "J_head_yaw", "J_head_pitch", "J_waist_pitch", "J_waist_roll", "J_waist_yaw",
    "J_hip_l_roll", "J_hip_l_yaw", "J_hip_l_pitch", "J_knee_l_pitch", "J_ankle_l_pitch", "J_ankle_l_roll",        
    "J_hip_r_roll", "J_hip_r_yaw", "J_hip_r_pitch", "J_knee_r_pitch", "J_ankle_r_pitch", "J_ankle_r_roll"};

const   double  dt = 0.001;
const   double  dt_200Hz = 0.005;
// MuJoCo load and compile model
char error[1000] = "Could not load binary model";
std::string pack_path = ros::package::getPath("azureloong_control");
mjModel* mj_model = mj_loadXML((pack_path+"/models/scene.xml").c_str(), 0, error, 1000);
mjData* mj_data = mj_makeData(mj_model);

double xyt_des[3] = {0.0, 0.0, 0.0};  // desired x, y, theta velocity
void velCmdCallback(const geometry_msgs::Twist & msg) {
    xyt_des[0] = msg.linear.x;
    xyt_des[1] = msg.linear.y;
    xyt_des[2] = msg.angular.z;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "azureloong_ctrl");
    ros::NodeHandle n;
    ros::Subscriber vel_sub;
    ros::Publisher js_pub, imu_pub, odom_pub;
          
    vel_sub = n.subscribe("cmd_vel", 10, velCmdCallback);
    js_pub = n.advertise<sensor_msgs::JointState>("joint_states", 10);
    imu_pub = n.advertise<sensor_msgs::Imu>("imu", 10);
    odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10);
    tf::TransformBroadcaster odom_br;
    
    // initialize classes
    UIctr uiController(mj_model,mj_data);   // UI control for Mujoco
    MJ_Interface mj_interface(mj_model, mj_data); // data interface for Mujoco
    Pin_KinDyn kinDynSolver(pack_path+"/models/AzureLoong.urdf"); // kinematics and dynamics solver
    DataBus robotState(kinDynSolver.model_nv); // data bus
    WBC_priority WBC_solv(kinDynSolver.model_nv, 18, 22, 0.7, mj_model->opt.timestep); // WBC solver
    MPC MPC_solv(dt_200Hz);  // mpc controller
    GaitScheduler gaitScheduler(0.25, mj_model->opt.timestep); // gait scheduler
    PVT_Ctr pvtCtr(mj_model->opt.timestep, (pack_path+"/common/joint_ctrl_config.json").c_str());// PVT joint control
    FootPlacement footPlacement; // foot-placement planner
    JoyStickInterpreter jsInterp(mj_model->opt.timestep); // desired baselink velocity generator

    // initialize UI: GLFW
    uiController.iniGLFW();
    uiController.enableTracking(); // enable viewpoint tracking of the body 1 of the robot
    uiController.createWindow("Demo",false); // NOTE: if the saveVideo is set to true, the raw recorded file could be 2.5 GB for 15 seconds!

    // initialize variables
    double stand_legLength = 1.01;//0.97;// desired baselink height
    double foot_height =0.07; // distance between the foot ankel joint and the bottom
    int model_nv=kinDynSolver.model_nv;

    robotState.width_hips = 0.229;
    footPlacement.kp_vx = 0.03;
    footPlacement.kp_vy = 0.03;
    footPlacement.kp_wz = 0.03;
    footPlacement.stepHeight = 0.15;
    footPlacement.legLength=stand_legLength;

    mju_copy(mj_data->qpos, mj_model->key_qpos, mj_model->nq*1); // set ini pos in Mujoco

    std::vector<double> motors_pos_des(model_nv - 6, 0);
    std::vector<double> motors_pos_cur(model_nv - 6, 0);
    std::vector<double> motors_vel_des(model_nv - 6, 0);
    std::vector<double> motors_vel_cur(model_nv - 6, 0);
    std::vector<double> motors_tau_des(model_nv - 6, 0);
    std::vector<double> motors_tau_cur(model_nv - 6, 0);

    // ini position and posture for foot-end and hand
    Eigen::Vector3d fe_l_pos_L_des={-0.018, 0.113, -stand_legLength};
    Eigen::Vector3d fe_r_pos_L_des={-0.018, -0.116, -stand_legLength};
    Eigen::Vector3d fe_l_eul_L_des={-0.000, -0.008, -0.000};
    Eigen::Vector3d fe_r_eul_L_des={0.000, -0.008, 0.000};
    Eigen::Matrix3d fe_l_rot_des= eul2Rot(fe_l_eul_L_des(0),fe_l_eul_L_des(1),fe_l_eul_L_des(2));
    Eigen::Matrix3d fe_r_rot_des= eul2Rot(fe_r_eul_L_des(0),fe_r_eul_L_des(1),fe_r_eul_L_des(2));

    Eigen::Vector3d hd_l_pos_L_des={-0.02, 0.32, -0.159};
    Eigen::Vector3d hd_r_pos_L_des={-0.02, -0.32, -0.159};
    Eigen::Vector3d hd_l_eul_L_des={-1.7581, 0.2129, 2.9581};
    Eigen::Vector3d hd_r_eul_L_des={1.7581, 0.2129, -2.9581};
    Eigen::Matrix3d hd_l_rot_des= eul2Rot(hd_l_eul_L_des(0),hd_l_eul_L_des(1),hd_l_eul_L_des(2));
    Eigen::Matrix3d hd_r_rot_des= eul2Rot(hd_r_eul_L_des(0),hd_r_eul_L_des(1),hd_r_eul_L_des(2));

    auto resLeg=kinDynSolver.computeInK_Leg(fe_l_rot_des,fe_l_pos_L_des,fe_r_rot_des,fe_r_pos_L_des);
    auto resHand=kinDynSolver.computeInK_Hand(hd_l_rot_des,hd_l_pos_L_des,hd_r_rot_des,hd_r_pos_L_des);
    Eigen::VectorXd qIniDes=Eigen::VectorXd::Zero(mj_model->nq, 1);
    qIniDes.block(7, 0, mj_model->nq - 7, 1)= resLeg.jointPosRes + resHand.jointPosRes;
    WBC_solv.setQini(qIniDes,robotState.q);

    //// -------------------------- main loop --------------------------------

    int  MPC_count = 0; // count for controlling the mpc running period

    double startSteppingTime=1;
    double startWalkingTime=2;

    mjtNum simstart = mj_data->time;
    double simTime = mj_data->time;

    while (!glfwWindowShouldClose(uiController.window) && ros::ok()) {
        simstart = mj_data->time;
        while (mj_data->time - simstart < 1.0 / 60.0 && uiController.runSim) { // press "1" to pause and resume, "2" to step the simulation
            mj_step(mj_model, mj_data);
            simTime=mj_data->time;
            ros::spinOnce();
            // Read the sensors:
            mj_interface.updateSensorValues();
            mj_interface.dataBusWrite(robotState);
            
            sensor_msgs::JointState js; 
            js.header.stamp = ros::Time::now();          
            for(int i = 0; i < JointName.size(); i++) {
                js.name.push_back(JointName[i]);
                js.position.push_back(robotState.motors_pos_cur[i]);
                js.velocity.push_back(robotState.motors_vel_cur[i]);
            }
            js_pub.publish(js);
            
            sensor_msgs::Imu imu;
            imu.header.stamp = ros::Time::now();
            imu.header.frame_id = "base";
            imu.linear_acceleration.x = robotState.baseAcc[0];
            imu.linear_acceleration.y = robotState.baseAcc[1];
            imu.linear_acceleration.z = robotState.baseAcc[2];
            imu.angular_velocity.x = robotState.baseAngVel[0];
            imu.angular_velocity.y = robotState.baseAngVel[1];
            imu.angular_velocity.z = robotState.baseAngVel[2];
            tf2::Quaternion quat;
            quat.setRPY(robotState.rpy[0], robotState.rpy[1], robotState.rpy[2]);
            quat = quat.normalize();
            imu.orientation.x = quat.getX(); 
            imu.orientation.y = quat.getY();
            imu.orientation.z = quat.getZ();
            imu.orientation.w = quat.getW();            
            imu_pub.publish(imu);
            
            nav_msgs::Odometry odom;
            odom.header.stamp = ros::Time::now();
            odom.header.frame_id = "odom";
            odom.pose.pose.position.x = robotState.basePos[0];
            odom.pose.pose.position.y = robotState.basePos[1];
            odom.pose.pose.position.z = robotState.basePos[2];
            geometry_msgs::Quaternion odom_quat = tf2::toMsg(quat);            
            odom.pose.pose.orientation = odom_quat;
            odom.child_frame_id = "base";
            odom.twist.twist.linear.x = robotState.baseLinVel[0];
            odom.twist.twist.linear.y = robotState.baseLinVel[1];
            odom.twist.twist.linear.z = robotState.baseLinVel[2];
            odom.twist.twist.angular.x = robotState.baseAngVel[0];
            odom.twist.twist.angular.y = robotState.baseAngVel[0];
            odom.twist.twist.angular.z = robotState.baseAngVel[0];           
            odom_pub.publish(odom);
            
            geometry_msgs::TransformStamped odom_trans;
            odom_trans.header.stamp = ros::Time::now();
            odom_trans.header.frame_id = "odom";
            odom_trans.child_frame_id = "base";
            odom_trans.transform.translation.x = robotState.basePos[0];
            odom_trans.transform.translation.y = robotState.basePos[1];
            odom_trans.transform.translation.z = robotState.basePos[2];
            odom_trans.transform.rotation = odom_quat;
            odom_br.sendTransform(odom_trans);
            
            // update kinematics and dynamics info
            kinDynSolver.dataBusRead(robotState);
            kinDynSolver.computeJ_dJ();
            kinDynSolver.computeDyn();
            kinDynSolver.dataBusWrite(robotState);

            // joint number: arm-l: 0-6, arm-r: 7-13, head: 14, waist: 15-17, leg-l: 18-23, leg-r: 24-29

            if (simTime > startWalkingTime) {
                jsInterp.setVxDesLPara(xyt_des[0], 2.0);
                jsInterp.setVyDesLPara(xyt_des[1], 1.0);
                jsInterp.setWzDesLPara(xyt_des[2], 1.0);
	        robotState.motionState = DataBus::Walk; // start walking
            } else
                jsInterp.setIniPos(robotState.q(0), robotState.q(1), robotState.base_rpy(2));
            jsInterp.step();
            robotState.js_pos_des(2) = stand_legLength + foot_height; // pos z is not assigned in jyInterp
            jsInterp.dataBusWrite(robotState); // only pos x, pos y, theta z, vel x, vel y , omega z are rewrote.

            if (simTime >= startSteppingTime) {
                // gait scheduler
                gaitScheduler.dataBusRead(robotState);
                gaitScheduler.step();
                gaitScheduler.dataBusWrite(robotState);

                footPlacement.dataBusRead(robotState);
                footPlacement.getSwingPos();
                footPlacement.dataBusWrite(robotState);
            }

            // ------------- MPC ------------
			MPC_count = MPC_count + 1;
            if (MPC_count > (dt_200Hz / dt-1)) {
                MPC_solv.dataBusRead(robotState);
                MPC_solv.cal();
                MPC_solv.dataBusWrite(robotState);
                MPC_count = 0;
            }

            // ------------- WBC ------------
            // WBC Calculation
            WBC_solv.dataBusRead(robotState);
            WBC_solv.computeDdq(kinDynSolver);
            WBC_solv.computeTau();
            WBC_solv.dataBusWrite(robotState);
            // get the final joint command
            if (simTime <= startSteppingTime) {
                robotState.motors_pos_des = eigen2std(resLeg.jointPosRes + resHand.jointPosRes);
                robotState.motors_vel_des = motors_vel_des;
                robotState.motors_tor_des = motors_tau_des;
            } else {
                MPC_solv.enable();
                Eigen::Matrix<double, 1, nx>  L_diag;
                Eigen::Matrix<double, 1, nu>  K_diag;
                L_diag <<
                       1.0, 1.0, 1.0,//eul
                        1.0, 200.0,  1.0,//pCoM
                        1e-7, 1e-7, 1e-7,//w
                        100.0, 10.0, 1.0;//vCoM
                K_diag <<
                       1.0, 1.0, 1.0,//fl
                        1.0, 1.0, 1.0,
                        1.0, 1.0, 1.0,//fr
                        1.0, 1.0, 1.0,1.0;
                MPC_solv.set_weight(1e-6, L_diag, K_diag);

                Eigen::VectorXd pos_des = kinDynSolver.integrateDIY(robotState.q, robotState.wbc_delta_q_final);
                robotState.motors_pos_des = eigen2std(pos_des.block(7, 0, model_nv - 6, 1));
                robotState.motors_vel_des = eigen2std(robotState.wbc_dq_final);
                robotState.motors_tor_des = eigen2std(robotState.wbc_tauJointRes);
            }

            // joint PVT controller
            pvtCtr.dataBusRead(robotState);
            if (simTime <= 3) {
                pvtCtr.calMotorsPVT(100.0 / 1000.0 / 180.0 * 3.1415);
            } else {
                pvtCtr.setJointPD(100,10,"J_ankle_l_pitch");
                pvtCtr.setJointPD(100,10,"J_ankle_l_roll");
                pvtCtr.setJointPD(100,10,"J_ankle_r_pitch");
                pvtCtr.setJointPD(100,10,"J_ankle_r_roll");
                pvtCtr.setJointPD(1000,100,"J_knee_l_pitch");
                pvtCtr.setJointPD(1000,100,"J_knee_r_pitch");
                pvtCtr.calMotorsPVT();
            }
            pvtCtr.dataBusWrite(robotState);

            // give the joint torque command to Webots
            mj_interface.setMotorsTorque(robotState.motors_tor_out);
        }
        uiController.updateScene();
    };
    // free visualization storage
    uiController.Close();

    return 0;
}
