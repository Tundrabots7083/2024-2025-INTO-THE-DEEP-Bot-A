package org.firstinspires.ftc.teamcode.ftc7083.subsystem;


import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.hardware.Servo;

/**
 * Wrist implements the wrist on the arm.
 */
public class Wrist extends SubsystemBase {

    Telemetry telemetry;

    double pitch = 0.0;
    double yaw = 0.0;
    double scorePosition = 90;
    Servo pitchServo;
    Servo yawServo;
    Servo rollServo;

    /**
     * Wrist initializes a new wrist as well as initializing all servos to be used.
     *
     * @param hardwareMap the hardware map that contains the servo hardware.
     * @param telemetry   the telemetry used to display data on the driver station.
     */
    public Wrist(@NonNull HardwareMap hardwareMap, @NonNull Telemetry telemetry) {
        this.telemetry = telemetry;
        pitchServo = new Servo(hardwareMap, "wristPitchServo", 180);
        yawServo = new Servo(hardwareMap, "wristYawServo", 180);
    }

    /**
     * setPitch sets the target for the pitch servo.
     *
     * @param pitch the final pitch target in degrees.
     * @return the set pitch.
     */
    public double setPitch(double pitch) {
        this.pitch = pitch;
        pitchServo.setDegrees(this.pitch);
        telemetry.addData("Wrist pitch: ", this.pitch);
        return this.pitch;
    }

    /**
     * setYaw sets the target for the yaw servo.
     *
     * @param yaw the final yaw target in degrees.
     * @return the set yaw.
     */
    public double setYaw(double yaw) {
        this.yaw = yaw;
        yawServo.setDegrees(this.yaw);
        telemetry.addData("Wrist yaw: ", this.yaw);
        return this.yaw;
    }
}
