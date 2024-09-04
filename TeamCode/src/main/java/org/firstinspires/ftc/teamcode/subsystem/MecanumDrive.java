package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.checkerframework.checker.nullness.qual.NonNull;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;
import java.util.Collection;

/**
 * MecanumDrive implements the drive chassis for the robot.
 */
@Config
public class MecanumDrive extends SubsystemBase {
    public double POWER_EXPONENT = 2.0;
    public double STRAFING_ADJUSTMENT = 1.1;

    private final Telemetry telemetry;
    private final DcMotorEx rightFront, rightRear, leftFront, leftRear;

    /**
     * MecanumDrive initializes a new mecanum drive train.
     *
     * @param hardwareMap the hardware map that contains the drone launcher hardware.
     * @param telemetry   the telemetry used to display data on the driver station.
     */
    public MecanumDrive(@NonNull HardwareMap hardwareMap, @NonNull Telemetry telemetry) {
        this.telemetry = telemetry;

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

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
    private void initMotor(@NonNull DcMotorEx motor) {
        MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        motor.setMotorType(motorConfigurationType);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
        // Adjust the values X, Y and turn to reduce the sensitivity of the controllers on the
        // low-end
        x = adjustValue(x * STRAFING_ADJUSTMENT); // Adjust for imperfect strafing
        y = adjustValue(y);
        turn = adjustValue(turn);

        double theta = Math.atan2(y, x);
        double power = Math.hypot(x, y);

        double sin = Math.sin(theta - Math.PI / 4);
        double cos = Math.cos(theta - Math.PI / 4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        double leftFrontPower = power * cos / max - turn;
        double rightFrontPower = power * sin / max + turn;
        double leftRearPower = power * sin / max - turn;
        double rightRearPower = power * cos / max + turn;

        // Normalize motor powers to ensure none exceeds 1.0
        double maxMotorPower = power + Math.abs(turn);
        if (maxMotorPower > 1) {
            leftFrontPower /= maxMotorPower;
            rightFrontPower /= maxMotorPower;
            leftRearPower /= maxMotorPower;
            rightRearPower /= maxMotorPower;
        }

        setMotorPowers(leftFrontPower, leftRearPower, rightRearPower, rightFrontPower);
    }

    /**
     * Adjust the value to reduce sensitivity of the joystick when pushed only a small distance
     *
     * @param value the value based from the controller
     * @return the adjusted value
     */
    private double adjustValue(double value) {
        // Raise the value to the desired power to provide values on an exponential curve.
        // If the original value was negative bu the new value raised to the exponent is positive,
        // negate the new value.
        double newValue = Math.pow(value, POWER_EXPONENT);
        if (value < 0 && newValue > 0) {
            newValue -= 1;
        }
        return newValue;
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
        double maxPower = maxAbs(1.0, leftFrontPower, leftRearPower, rightFrontPower, rightRearPower);

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

        telemetry.addData("[DRIVE] Left Front Power", leftFrontPower);
        telemetry.addData("[DRIVE] Left Rear Power", leftRearPower);
        telemetry.addData("[DRIVE] Right Front Power", rightFrontPower);
        telemetry.addData("[DRIVE] Right Rear Power", rightRearPower);
    }

    /**
     * Gets a string representation of this mecanum drive.
     *
     * @return a string representation of this mecanum drive
     */
    @NonNull
    @Override
    public String toString() {
        return "MecanumDrive{" +
                "rightFront=" + rightFront +
                ", rightRear=" + rightRear +
                ", leftFront=" + leftFront +
                ", leftRear=" + leftRear +
                '}';
    }
}