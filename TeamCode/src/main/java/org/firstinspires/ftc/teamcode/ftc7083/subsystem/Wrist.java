package org.firstinspires.ftc.teamcode.ftc7083.subsystem;


import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.hardware.Servo;

public class Wrist extends SubsystemBase {

    Telemetry telemetry;

    double pitch = 0.0;
    double yaw = 0.0;
    double roll = 0.0;
    double scorePosition = 90;
    Servo wristPitchServo;
    Servo wristYawServo;
    Servo wristRollServo;

    public Wrist(@NonNull HardwareMap hardwareMap, @NonNull Telemetry telemetry) {
        this.telemetry = telemetry;
        Servo wristPitchServo = new Servo(hardwareMap, "wristPitchServo", 180);
        Servo wristYawServo = new Servo(hardwareMap, "wristYawServo", 180);
        Servo wristRollServo = new Servo(hardwareMap, "wristRollServo", 180);
    }

    public double setPitch(double pitch) {
        this.pitch = pitch;
        wristPitchServo.setDegrees(this.pitch);
        telemetry.addData("Wrist pitch: ", this.pitch);
        return this.pitch;
    }

    public double setYaw(double yaw) {
        this.yaw = yaw;
        wristYawServo.setDegrees(this.yaw);
        telemetry.addData("Wrist yaw: ", this.yaw);
        return this.yaw;
    }

    public double setRoll(double roll) {
        this.roll = roll;
        wristRollServo.setDegrees(this.roll);
        telemetry.addData("Wrist roll: ", this.roll);
        return this.roll;
    }

}
