#ifndef PID_H
#define PID_H

class PID {
    private:
        float kp;
        float ki;
        float kd;

        //Output is always between [-output_limit, output_limit]
        float output_limit;

        
        float integral;
        float last_measurement;
        bool first_call; //True if there is no last_measurement yet

        float limit(float value);

    public:
        PID();
        PID(float kp_, float ki_, float kd_, float output_limit_);

        //dt: Seconds since last call
        float calculate_PID(float target, float measurement, float dt);

        void changeConstants(float kp_, float ki_, float kd_);
        void reset();
};

#endif