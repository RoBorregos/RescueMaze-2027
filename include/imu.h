#ifndef IMU_H
#define IMU_H

class Imu {
    private:
        bool ready; //True if BNO answered at startup
        bool has_reading; //True after the first heading arrives
        float last_raw_deg; //Last heading -180 to 180 degrees
        float heading_deg; //Continuous, does not jump to 180

        void enableReports();

    public:
        bool begin(); //Returns false if BNO doesnt answer
        void update(); //Reads news data, calls every control cycle
        bool isReady() const; //True if there is a valid heading
        float getHeading() const; //Degrees, clockwise positive, 0 at startup
};

#endif