#ifndef SPEED_CONTROLLER
#define SPEED_CONTROLLER

#include <Romi32U4.h>
#include "Position_estimation.h"

class SpeedController{
    private:
        const float Kp = 0.5; //Adapt the parameters until your robot moves at the speed you command it to drive
        const float Ki = 0.1;
        const float Kitheta = .1;
        const float Kpd = 1000;
        const float Kptheta = 1050;
        float E_left = 0; 
        float E_right = 0;
        int counts = 1440; //number of counts for a 180 degree turn; you will likely have to change this
        float error_distance = 0;
        float error_theta_sum = 0;
        float error_dist_sum  = 0;
        float kp_wall = 9;
        float kd_wall = 4.5;
        float prev_e_distance = 0;
        float base_speed = 100;
        
    
    public:
        struct constrained_acceleration {
            float constrained_velocity_left;
            float constrained_velocity_right;

        };
        Position odometry;

        
        void Init(void);
        void Run(float, float); 
        boolean Turn(int,int); //degrees, direction of rotation: 0->left, 1->right
        boolean Straight(int, int); //speed, duration
        boolean Curved(int,int,int); //speed left, speed right, duration
        boolean MoveToPosition(float,float); //target_x, target_y
        //boolean MoveWithAccelCap(float,float,int);
        void WallFollow(float);
        void Stop(void);
};

#endif