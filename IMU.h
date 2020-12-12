#ifndef IMU
#define IMU

#include <Romi32U4.h>

class IMU_sensor{
    private:
        int data[3] = {0};
        char report[120];
        
    public:
        struct acceleration_data {
            int X;
            int Y;
            int Z;
        };
        void Init(void);
        void PrintAcceleration(void);
        acceleration_data ReadAcceleration(void);
};

#endif