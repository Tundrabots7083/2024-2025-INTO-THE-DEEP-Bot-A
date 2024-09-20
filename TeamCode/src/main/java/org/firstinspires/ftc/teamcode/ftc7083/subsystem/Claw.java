package org.firstinspires.ftc.teamcode.ftc7083.subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ftc7083.hardware.Servo;

public class Claw extends SubsystemBase {
    private static final double DEFAULT_OPEN_DEGREES = 45.0;
    private final Servo servo;


    public Claw(HardwareMap hardwareMap, String deviceName, double maxDegrees) {
        servo = new Servo(hardwareMap, deviceName, maxDegrees);
        // todo: May want to set initial position and direction for servo.
    }
    public double open() {
        servo.setDegrees(DEFAULT_OPEN_DEGREES);
        return servo.getPosition();
    }

    public double open(double degrees) {
        servo.setDegrees(degrees);
        return servo.getPosition();
    }

    public double close() {
        servo.setDegrees(0);
        return servo.getPosition();
    }



}