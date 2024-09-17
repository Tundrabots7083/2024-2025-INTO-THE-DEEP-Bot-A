package org.firstinspires.ftc.teamcode.ftc7083.subsystem;


import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.hardware.Servo;

public class Wrist extends SubsystemBase {

    Telemetry telemetry;

    double pitch = 0.0;
    double yaw = 0.0;
    double scorePosition = 90;

    public Wrist(@NonNull HardwareMap hardwareMap, @NonNull Telemetry telemetry) {
        this.telemetry = telemetry;
        Servo wristPitchServo = new Servo(hardwareMap, "wristPitchServo", 180);
        Servo wristYawServo = new Servo(hardwareMap, "wristYawServo", 180);
    }

    public double setPitch(double pitch) {
        this.pitch = pitch;
        telemetry.addData("Wrist pitch: ", this.pitch);
        return this.pitch;
    }

    public double setYaw(double yaw) {
        this.yaw = yaw;
        telemetry.addData("Wrist yaw: ", this.yaw);
        return this.yaw;
    }

    public double alignWristToHorizontal(double armAngle) {
        this.pitch = 180 - armAngle;
        return this.pitch;
    }
}
