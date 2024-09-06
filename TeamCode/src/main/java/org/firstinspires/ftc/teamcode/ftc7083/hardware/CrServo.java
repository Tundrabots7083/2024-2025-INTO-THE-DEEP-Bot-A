package org.firstinspires.ftc.teamcode.ftc7083.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoController;

public class CrServo implements CRServo, PwmControl {
    private final CRServoImplEx servoImpl;

    /**
     * Instantiate a new continuous rotation servo for the robot.
     *
     * @param hardwareMap the mapping for all hardware on the robot
     * @param deviceName  the name of the continuous rotation servo  as configured via the Driver Station
     */
    public CrServo(HardwareMap hardwareMap, String deviceName) {
        servoImpl = (CRServoImplEx) hardwareMap.get(CRServo.class, deviceName);
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
    public void setDirection(Direction direction) {
        servoImpl.setDirection(direction);
    }

    @Override
    public Direction getDirection() {
        return servoImpl.getDirection();
    }

    @Override
    public void setPower(double power) {
        servoImpl.setPower(power);
    }

    @Override
    public double getPower() {
        return servoImpl.getPower();
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

    @Override
    public void setPwmRange(PwmRange range) {
        servoImpl.setPwmRange(range);
    }

    @Override
    public PwmRange getPwmRange() {
        return servoImpl.getPwmRange();
    }

    @Override
    public void setPwmEnable() {
        servoImpl.setPwmEnable();
    }

    @Override
    public void setPwmDisable() {
        servoImpl.setPwmDisable();
    }

    @Override
    public boolean isPwmEnabled() {
        return servoImpl.isPwmEnabled();
    }
}
