package org.firstinspires.ftc.teamcode.ftc7083.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.teamcode.ftc7083.hardware.conversion.ServoConversion;

public class Servo implements com.qualcomm.robotcore.hardware.Servo {

    private final com.qualcomm.robotcore.hardware.Servo servoImpl;
    private final ServoConversion conv;

    /**
     * Instantiate a new motor for the robot.
     *
     * @param servoImpl  the servo as retrieved via the hardware map
     * @param maxDegrees the maximum number of degrees to which the servo can rotate
     */
    protected Servo(com.qualcomm.robotcore.hardware.Servo servoImpl, double maxDegrees) {
        this.servoImpl = servoImpl;
        conv = new ServoConversion(maxDegrees);
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
     * Gets the current degree offset of the servo.
     *
     * @return the current degree offset of the servo
     */
    public double getDegrees() {
        return conv.positionToDegrees(getPosition());
    }

    /**
     * Sets the position of the servo to the specified number of degrees.
     *
     * @param degrees the degrees to which to set the servo
     */
    public void setDegrees(double degrees) {
        double position = conv.degreesToPosition(degrees);
        servoImpl.setPosition(position);
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
