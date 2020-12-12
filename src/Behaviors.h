#ifndef BEHAVIORS
#define BEHAVIORS

#include <Romi32U4.h>

class Behaviors{
    private:
        int threshold = 550;
        int threshold_ramp = 1700;
        int data[3] = {0};
        enum ROBOT_STATE {IDLE, DRIVE_DISTANCE, TURN, RAMP, WALL_FOLLOW, COLLSION_DETECTION,BACK_UP};
        ROBOT_STATE robot_state = IDLE; //initial state: IDLE
        float i = 0;
        float wall_follow_distance = 15; //cm
        float distance_at_end = .1; // m
        long prev_time = 0;
        int ramp_counter = 0;
        long start_time = 0;
        long initial_cause =0;
        
         
    public:
        void Init(void);
        void Stop(void);
        void Run(void);
        bool DetectCollision(void);
        bool DetectRamp(void);
};
#endif
