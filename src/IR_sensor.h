#ifndef IR_SENSOR
#define IR_SENSOR

#include <Romi32U4.h>

class IRsensor{
    private:
        const int pin_IR = A0;
    public:
        void Init(void);
        float ReadData(void);
        float PrintData(void);
};

#endif