package org.firstinspires.ftc.teamcode.ftc7083.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoController;

import java.math.BigDecimal;
import java.math.MathContext;

public class Servo implements com.qualcomm.robotcore.hardware.Servo {
    private static final BigDecimal MAX_POSITION = new BigDecimal("1.0");
    private static double DEFAULT_MAX_DEGREES = 120.0;

    private final com.qualcomm.robotcore.hardware.Servo servoImpl;

    private BigDecimal maxDegrees = new BigDecimal(DEFAULT_MAX_DEGREES);

    /**
     * Instantiate a new motor for the robot.
     *
     * @param servoImpl  the servo as retrieved via the hardware map
     * @param maxDegrees the maximum number of degrees to which the servo can rotate
     */
    protected Servo(com.qualcomm.robotcore.hardware.Servo servoImpl, double maxDegrees) {
        this.servoImpl = servoImpl;
    }

    /**
     * Instantiate a new servo for the robot.
     *
     * @param hardwareMap the mapping for all hardware on the robot
     * @param deviceName  the name of the servo as configured via the Driver Station
     * @param maxDegrees  the maximum number of degrees to which the servo can rotate
     */
    public Servo(HardwareMap hardwareMap, String deviceName, double maxDegrees) {
        this(hardwareMap.get(Servo.class, deviceName), maxDegrees);
    }

    @Override
    public ServoController getController() {
        return servoImpl.getController();
    }

    @Override
    public int getPortNumber() {
        return servoImpl.getPortNumber();
    }

    @Override
    public Direction getDirection() {
        return servoImpl.getDirection();
    }

    @Override
    public void setDirection(Direction direction) {
        servoImpl.setDirection(direction);
    }

    /**
     * Sets the maximum number of degrees this servo may rotate.
     *
     * @param maxDegrees the maximum number of degrees this servo may rotate
     * @return this servo
     */
    public Servo setMaxDegrees(int maxDegrees) {
        this.maxDegrees = new BigDecimal(maxDegrees);
        return this;
    }

    /**
     * Gets the current degree offset of the servo.
     *
     * @return the current degree offset of the servo
     */
    public double getDegrees() {
        BigDecimal pos = BigDecimal.valueOf(getPosition());
        BigDecimal degrees = pos.multiply(maxDegrees);
        return degrees.doubleValue();
    }

    /**
     * Sets the position of the servo to the specified number of degrees.
     *
     * @param degrees the degrees to which to set the servo
     */
    public void setDegrees(double degrees) {
        BigDecimal deg = new BigDecimal(degrees);
        BigDecimal position = deg.divide(maxDegrees, MathContext.DECIMAL128);
        servoImpl.setPosition(position.doubleValue());
        servoImpl.setPosition(deg.doubleValue());
    }

    @Override
    public double getPosition() {
        return servoImpl.getPosition();
    }

    @Override
    public void setPosition(double position) {
        servoImpl.setPosition(position);
    }

    @Override
    public void scaleRange(double min, double max) {
        servoImpl.scaleRange(min, max);
    }

    @Override
    public Manufacturer getManufacturer() {
        return servoImpl.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return servoImpl.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return servoImpl.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return servoImpl.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        servoImpl.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        servoImpl.close();
    }
}
