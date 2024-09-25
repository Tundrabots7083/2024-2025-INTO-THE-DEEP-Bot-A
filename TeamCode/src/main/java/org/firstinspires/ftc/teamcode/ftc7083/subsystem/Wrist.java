package org.firstinspires.ftc.teamcode.ftc7083.subsystem;


import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.hardware.Servo;

/**
 * Wrist implements the servo operated wrist on the arm.
 */
public class Wrist extends SubsystemBase {

    Telemetry telemetry;

    double pitch = 0.0;
    double yaw = 0.0;
    double scorePosition = 90;
    Servo frontServo;
    Servo backServo;

    /**
     * Wrist initializes a new wrist as well as initializing all servos to be used.
     *
     * @param hardwareMap the hardware map that contains the servo hardware.
     * @param telemetry   the telemetry used to display data on the driver station.
     */
    public Wrist(@NonNull HardwareMap hardwareMap, @NonNull Telemetry telemetry) {
        this.telemetry = telemetry;
        frontServo = new Servo(hardwareMap, "wristFrontServo");
        frontServo.setMaxDegrees(180);
        backServo = new Servo(hardwareMap, "wristBackServo");
        backServo.setMaxDegrees(180);
    }

    /**
     * setPitch sets the target for the pitch servo.
     *
     * @param pitch the final pitch target in degrees.
     */
    public void setPitch(double pitch) {
        double maxPitch;
        maxPitch = pitch > 0 ? 40 : -50;
        this.pitch = pitch <= 40 && pitch >= -50 ? pitch : maxPitch;
        double frontServoPitch = 90 - pitch;
        double backServoPitch = 90 + pitch;

        frontServo.setDegrees(frontServoPitch);
        backServo.setDegrees(backServoPitch);
      
        telemetry.addData("Wrist pitch: ", this.pitch);
        telemetry.update();
    }

    /**
     * setYaw sets the target for the yaw servo.
     *
     * @param yaw the final yaw target in degrees.
     */
    public void setYaw(double yaw) {
        this.yaw = maxAbs(yaw,80) <= 80 ? yaw : 80;
        double frontServoYaw = 90 -  yaw;
        double backServoYaw = 90 - yaw;

        frontServo.setDegrees(frontServoYaw);
        backServo.setDegrees(backServoYaw);

        telemetry.addData("Wrist yaw: ", this.yaw);
        telemetry.update();
    }

    /**
     * This method calculates the values to assign to the frontServo and backServo
     * given the pitch and yaw and sets the servos to their respective values.
     *
     * @param pitch wrist pitch
     * @param yaw wrist yaw
     */
    public void setPosition(double pitch, double yaw) {
        this.yaw = maxAbs(yaw,80) <= 80 ? yaw : 80;

        double maxPitch;
        maxPitch = pitch > 0 ? 40 : -50;
        this.pitch = pitch <= 40 && pitch >= -50 ? pitch : maxPitch;

        double frontServoYaw = 90 -  yaw;
        double backServoYaw = 90 - yaw;

        double frontServoPosition = (frontServoYaw - pitch) <= 180 ? (frontServoYaw - pitch) : 180;
        double backServoPosition = (backServoYaw + pitch) <= 180 ? (backServoYaw + pitch) : 180;

        frontServo.setDegrees(frontServoPosition);
        backServo.setDegrees(backServoPosition);
    }

    /**
     * getPitchPosition returns the current set pitch position.
     *
     * @return returns pitch position
     */
    public double getPitchPosition () {
        return this.pitch;
    }

    /**
     * getYawPosition returns the current set yaw position.
     *
     * @return returns yaw position
     */
    public double getYawPosition () {
        return this.yaw;
    }

    /**
     * Returns the position of the front servo in degrees.
     * @return frontServo degrees
     */
    public double getFrontServoDegrees () {return frontServo.getDegrees();}

    /**
     * Returns the position of the back servo in degrees.
     * @return backServo degrees
     */
    public double getBackServoDegrees () {return backServo.getDegrees();}
}
