#include <Romi32U4.h>
#include "Encoders.h"
#include "Speed_controller.h"
#include "Position_estimation.h"
#include "IR_sensor.h"


Romi32U4Motors motors;
Encoder MagneticEncoder; 
//Position odometry;
IRsensor SharpIR;

float time_track = 0;

void SpeedController::Init(void)
{

    SharpIR.Init();
    MagneticEncoder.Init();
    odometry.Init();
}

void SpeedController::Run(float target_velocity_left, float target_velocity_right)
{
    if(MagneticEncoder.UpdateEncoderCounts()){
        time_track = time_track + 50/1000.0;
        float e_left = target_velocity_left - MagneticEncoder.ReadVelocityLeft();
        float e_right = target_velocity_right - MagneticEncoder.ReadVelocityRight();
        
        E_left += e_left;
        E_right += e_right;

        float u_left = Kp*e_left + Ki*E_left;
        float u_right = Kp*e_right + Ki*E_right;

        motors.setEfforts(u_left, u_right);
        odometry.UpdatePose(target_velocity_left, target_velocity_right);
    }
}

boolean SpeedController::MoveToPosition(float target_x, float target_y)
{
    do
    {    
        float error_x = target_x - odometry.ReadPose().X;
        float error_y = target_y - odometry.ReadPose().Y;
        error_distance = sqrt(pow(error_x, 2) + pow(error_y, 2));
        float error_theta = atan2(error_y, error_x) - odometry.ReadPose().THETA;
        if(error_theta > (PI/180)*185) error_theta -= 2*PI;
        else if(error_theta < -(PI/180)*185) error_theta += 2*PI;
        error_theta_sum += error_theta;
        error_dist_sum += error_distance;

        //float u_left = constrain(Kpd*error_distance - Kptheta*error_theta, -300, 300);
        //float u_right = constrain(Kpd*error_distance + Kptheta*error_theta, -300, 300);

        float u_left = constrain(Kpd*error_distance + Ki*error_dist_sum - Kptheta*error_theta + Kitheta*error_theta_sum, -300, 300);
        float u_right = constrain(Kpd*error_distance + Ki*error_dist_sum + Kptheta*error_theta + Kitheta*error_theta_sum, -300, 300);

        Run(u_left, u_right);
        odometry.PrintPose();
    } 
    while (error_distance >= .03); //define a distance criteria that lets the robot know that it reached the waypoint.
    error_theta_sum = 0;
    error_dist_sum = 0;
    return 1;
}

boolean SpeedController::Turn(int degree, int direction)
{
    motors.setEfforts(0, 0);
    int turns = counts*(degree/180.0);
    int count_turn = MagneticEncoder.ReadEncoderCountLeft();

    while(abs(abs(count_turn) - abs(MagneticEncoder.ReadEncoderCountLeft())) <= turns)
    {
        if(!direction) Run(50,-50);
        else Run(-50,50);
    }
    motors.setEfforts(0, 0);
    return 1;
}

boolean SpeedController::Straight(int target_velocity, int time)
{
    motors.setEfforts(0, 0);
    unsigned long now = millis();

    while ((unsigned long)(millis() - now) <= time*1000){
        Run(target_velocity,target_velocity);
    }
    motors.setEfforts(0, 0);
    return 1;
}

boolean SpeedController::Curved(int target_velocity_left, int target_velocity_right, int time) //in mm/s and s
{
    motors.setEfforts(0, 0);
    unsigned long now = millis();

    while ((unsigned long)(millis() - now) <= time*1000){
        Run(target_velocity_left,target_velocity_right);
    }
    motors.setEfforts(0, 0);
    return 1;
}


void SpeedController::Stop()
{
    motors.setEfforts(0,0);
    odometry.Stop();
    time_track = 0;
}

void SpeedController::WallFollow(float target_distance)
{
  float new_read = SharpIR.ReadData();
  float speed = -1*(kp_wall*(target_distance - new_read)+ kd_wall *(new_read -prev_e_distance));
  new_read = prev_e_distance;
  Run(base_speed + speed, base_speed - speed);
  }