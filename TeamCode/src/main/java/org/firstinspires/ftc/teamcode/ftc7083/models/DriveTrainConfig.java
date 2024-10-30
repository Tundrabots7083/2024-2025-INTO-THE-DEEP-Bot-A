package org.firstinspires.ftc.teamcode.ftc7083.models;

public class DriveTrainConfig {    final double SPEED_GAIN  =  0.04  ;   //0.02  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    public double speedGain =  0.04  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)

    public double strafeGain =  0.03 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    public double turnGain =  0.02  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    public double maxAutoSpeed = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    public double maxAutoStrafe = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    public double maxAutoTurn = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    public DriveTrainConfig(){
         speedGain =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
         strafeGain =  0.015 ;   //   Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
        turnGain =  0.01  ;   //   Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

         maxAutoSpeed = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
         maxAutoStrafe = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
         maxAutoTurn = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
    }
}
