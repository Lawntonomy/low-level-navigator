#ifndef IMU_H
#define IMU_H

class IMU
{
  public:
    IMU();
    void init();
    float getYawRate(); // Returns angular velocity in rad/s
  private:
    float yaw_rate;
};

#endif
