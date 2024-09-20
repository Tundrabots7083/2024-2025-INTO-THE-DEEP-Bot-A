package org.firstinspires.ftc.teamcode.ftc7083.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ftc7083.hardware.Servo;

@Config
@TeleOp(name = "Servo Test", group = "tests")
public class ServoTest extends OpMode {
    public static String CLAW_PITCH_SERVO_NAME = "wristYawServo";
    // public static double SERVO_MAX_DEGREES = 270;
    public static double SERVO_MAX_DEGREES = 180;
    public static double SERVO_DEGREES = 0.0;

    private Servo servo;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        servo = new Servo(hardwareMap, CLAW_PITCH_SERVO_NAME);
        servo.setMaxDegrees(SERVO_MAX_DEGREES);

        telemetry.addLine("Initialization Complete");
        telemetry.update();
    }

    @Override
    public void loop() {
        servo.setMaxDegrees(SERVO_MAX_DEGREES);
        servo.setDegrees(SERVO_DEGREES);
        telemetry.addData("Target Degrees", SERVO_DEGREES);
        telemetry.addData("Retrieved Degrees", servo.getDegrees());
        telemetry.addData("Position", servo.getPosition());
        telemetry.update();
    }
}
