package org.firstinspires.ftc.teamcode.ftc7083.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoController;

/**
 * Instances of Servo provide access to servo hardware devices.
 */
public class Servo implements com.qualcomm.robotcore.hardware.Servo {
    private final com.qualcomm.robotcore.hardware.Servo servoImpl;
    private double maxDegrees;

    /**
     * Instantiates a new servo for the robot.
     *
     * @param servoImpl  the servo as retrieved via the hardware map
     */
    protected Servo(com.qualcomm.robotcore.hardware.Servo servoImpl) {
        this.servoImpl = servoImpl;
    }

    /**
     * Instantiates a new servo for the robot.
     *
     * @param hardwareMap the mapping for all hardware on the robot
     * @param deviceName  the name of the servo as configured via the Driver Station
     */
    public Servo(HardwareMap hardwareMap, String deviceName) {
        this(hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, deviceName));
    }

    /**
     * Gets the maximum number of degrees this servo may rotate.
     *
     * @return the maximum number of degrees this servo may rotate
     */
    public double getMaxDegrees() {
        return maxDegrees;
    }

    /**
     * Sets the maximum number of degrees this servo may rotate.
     *
     * @param maxDegrees the maximum number of degrees this servo may rotate
     */
    public void setMaxDegrees(double maxDegrees) {
        this.maxDegrees = maxDegrees;
    }

    /**
     * Gets the current degree offset of the servo.
     *
     * @return the current degree offset of the servo
     */
    public double getDegrees() {
        double position = getPosition();
        return position * maxDegrees;
    }

    /**
     * Sets the position of the servo to the specified number of degrees.
     *
     * @param degrees the degrees to which to set the servo
     */
    public void setDegrees(double degrees) {
        double position = degrees / maxDegrees;
        servoImpl.setPosition(position);
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
