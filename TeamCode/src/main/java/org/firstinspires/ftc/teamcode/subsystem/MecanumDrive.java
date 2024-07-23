package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;
import java.util.Collection;

/**
 * MecanumDrive implements the drive chassis for the robot.
 */
@Config
public class MecanumDrive extends SubsystemBaseEx {
    private final Telemetry telemetry;
    private final DcMotorEx rightFront, rightRear, leftFront, leftRear;

    /**
     * MecanumDrive initializes a new mecanum drive train.
     *
     * @param hardwareMap the hardware map that contains the drone launcher hardware.
     * @param telemetry   the telemetry used to display data on the driver station.
     */
    public MecanumDrive(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        Collection<DcMotorEx> motors = Arrays.asList(leftFront, leftRear, rightFront, rightRear);

        for (DcMotorEx motor : motors) {
            initMotor(motor);
        }
    }

    /**
     * initMotor initializes a motor attached to the mecanum wheel.
     *
     * @param motor the motor to be initialized.
     */
    private void initMotor(DcMotorEx motor) {
        MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        motor.setMotorType(motorConfigurationType);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    /**
     * drive sets the powers to the wheel motors to result in the robot moving
     * in the direction provided on input.
     *
     * @param x    how much to move right or, if a negative value, left.
     * @param y    how much to move forward or, if a negative value, backward.
     * @param turn how much to rotate the robot.
     */
    public void drive(double x, double y, double turn) {
        double theta = Math.atan2(y, x);
        double power = Math.hypot(x, y);

        double sin = Math.sin(theta - Math.PI / 4);
        double cos = Math.cos(theta - Math.PI / 4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        double leftFrontPower = power * cos / max - turn;
        double rightFrontPower = power * sin / max + turn;
        double leftRearPower = power * sin / max - turn;
        double rightRearPower = power * cos / max + turn;

        // Reduce the power until we hit a maximum amount
        if ((power + Math.abs(turn)) > 1) {
            leftFrontPower /= power + turn;
            rightFrontPower /= power + turn;
            leftRearPower /= power + turn;
            rightRearPower /= power + turn;
        }

        // Adjust the power to give a curve for ramping up power input
        leftFrontPower = getAdjustedPower(leftFrontPower);
        leftRearPower = getAdjustedPower(leftRearPower);
        rightRearPower = getAdjustedPower(rightRearPower);
        rightFrontPower = getAdjustedPower(rightFrontPower);

        setMotorPowers(leftFrontPower, leftRearPower, rightRearPower, rightFrontPower);
    }

    /**
     * Adjust the power to provide more granular acceleration
     *
     * @param power the power based on the control
     * @return the adjusted power
     */
    private double getAdjustedPower(double power) {
        double sign = 1;
        if (power < 0) {
            sign = -1;
        }
        return Math.pow(power, 2) * sign;
    }

    /**
     * setMotorPowers sets the power for the wheels, normalizing for a maximum power of 1.0.
     *
     * @param leftFrontPower  the power for the left front motor.
     * @param leftRearPower   the power for the left rear motor
     * @param rightRearPower  the power for the right rear motor.
     * @param rightFrontPower the power for the right front motor.
     */
    public void setMotorPowers(double leftFrontPower, double leftRearPower, double rightRearPower, double rightFrontPower) {
        // Get the maximum power for any motor, or 1.0, whichever is greater
        double maxPower = 1.0;
        maxPower = Math.max(maxPower, Math.abs(leftFrontPower));
        maxPower = Math.max(maxPower, Math.abs(leftRearPower));
        maxPower = Math.max(maxPower, Math.abs(rightFrontPower));
        maxPower = Math.max(maxPower, Math.abs(rightRearPower));

        // Divide by the maximum power, which guarantees no motor's power will exceed 1.0.
        // This also ensures that all motors get a proportional amount of power should the
        // input power for any motor exceed 1.0.
        leftFrontPower /= maxPower;
        leftRearPower /= maxPower;
        rightFrontPower /= maxPower;
        rightRearPower /= maxPower;

        // Now that the power have been normalized, go ahead and set power for the motors.
        leftFront.setPower(leftFrontPower);
        leftRear.setPower(leftRearPower);
        rightFront.setPower(rightFrontPower);
        rightRear.setPower(rightRearPower);

        Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();
        telemetry.addData("[DRIVE] Left Front Power", leftFrontPower);
        telemetry.addData("[DRIVE] Left Rear Power", leftRearPower);
        telemetry.addData("[DRIVE] Right Front Power", rightFrontPower);
        telemetry.addData("[DRIVE] Right Rear Power", rightRearPower);
    }
}
