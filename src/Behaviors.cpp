#include <Romi32U4.h>
#include "Behaviors.h"
#include "Speed_controller.h"
#include "Median_filter.h"
#include "Position_estimation.h"
#include "IMU.h"
#include "IR_sensor.h"

//motor-speed controller
SpeedController robots;

void Behaviors::Stop(void)
{
    robots.Stop();
}

//sensors
IMU_sensor LSM6;
Romi32U4ButtonA buttonA;
Romi32U4ButtonB buttonB;

//median filter
MedianFilter med_x;
MedianFilter med_y;
MedianFilter med_z;

//wall-following controller


//Position position;

void Behaviors::Init(void)
{
    Serial.begin(9600);
    LSM6.Init();
    med_x.Init();
    med_y.Init();
    med_z.Init();
    Serial.begin(9600);
    robots.Init();
}

boolean Behaviors::DetectCollision(void)
{
    auto data_acc = LSM6.ReadAcceleration();
    data[0] = med_x.Filter(data_acc.X)*0.061;
    data[1] = med_y.Filter(data_acc.Y)*0.061;
    data[2] = med_z.Filter(data_acc.Z)*0.061;
    if((abs(data[0]) > threshold) || (abs(data[1]) > threshold)) return 1;
    else return 0;
}

boolean Behaviors::DetectRamp(void)
{
    auto data_acc = LSM6.ReadAcceleration();
    data[0] = med_x.Filter(data_acc.X)*0.061;
    data[1] = med_y.Filter(data_acc.Y)*0.061;
    data[2] = med_z.Filter(data_acc.Z)*0.061;
    if(abs(data[2]) > threshold_ramp) return 1;
    else return 0;
}

void Behaviors::Run(void)
{
    
    switch (robot_state){
    case IDLE:
        if(millis() >= 50+prev_time){
            Serial.println("IDLE");
            prev_time = millis();
        }
        if((buttonA.getSingleDebouncedRelease()|| buttonB.getSingleDebouncedRelease())  && i == 0){ 
            delay(1000);
            i = 1;
            robot_state = COLLSION_DETECTION; 
            robots.Stop();             
        } 
        else if((buttonA.getSingleDebouncedRelease()|| buttonB.getSingleDebouncedRelease()) && i == 1){
            delay(1000);
            robot_state = TURN;
            robots.Stop();
        }
        else { 
            robot_state = IDLE;
            robots.Stop(); 
        }   
        break;
    
    case COLLSION_DETECTION:
        if(millis() >= 50+prev_time){
            Serial.println("COLLSION_DETECTION");
            prev_time = millis();
        }
        if(DetectCollision() == true){ 
            robot_state = BACK_UP; 
            robots.Stop();             
        } 
        else {
            robot_state = COLLSION_DETECTION;
            robots.Run(130, 130);
        }
        break;

    case BACK_UP:
        if(millis() >= 50+prev_time){
            Serial.println("Back Up");
            prev_time = millis();
        }
        robots.Straight(-50,3);
        robots.Stop();
        robot_state = IDLE;
        break;
        
    case TURN:
        if(millis() >= 50+prev_time){
            Serial.println("TURN");
            prev_time = millis();
        }
        robots.Turn(90, 1);
        robot_state = WALL_FOLLOW;
        break;
    
    case WALL_FOLLOW:
        if(millis() >= 50+prev_time){
            Serial.println("WALL_FOLLOW");
            prev_time = millis();
        }
        if(DetectRamp() && ramp_counter == 0){
            
            ramp_counter ++;
            robot_state = DRIVE_DISTANCE;
            //initial_cause = millis();
        }   
        else{
            robots.WallFollow(wall_follow_distance);
            robot_state = WALL_FOLLOW;
        }
        /*if(DetectRamp() && ramp_counter ==1 && millis()> initial_cause+200)
        {
            robots.Stop();
            robot_state = DRIVE_DISTANCE;
        }*/
        break;


    case DRIVE_DISTANCE:
        /*if(millis() >= 50+prev_time){
        
            Serial.println("DRIVE_DISTANCE");
            prev_time = millis();
        }        
        float error_distance = - distance_at_end + robots.odometry.ReadPose().X;
        if(millis() >= start_time +1500){
            robot_state = IDLE;
            robots.Stop();
        }
        else {
            robots.WallFollow(wall_follow_distance);
            robot_state = DRIVE_DISTANCE;
        }*/
        robots.Straight(115,3);
        robot_state = IDLE;
        break;
    };
    
   /*if(DetectRamp())
   {
       robots.Stop();
       delay(1000);
   }
   robots.WallFollow(wall_follow_distance);
   if(millis()%10 == 0)
   {
       auto data = LSM6.ReadAcceleration();
       Serial.println(med_z.Filter(data.Z)*0.061);
   }*/
   
}