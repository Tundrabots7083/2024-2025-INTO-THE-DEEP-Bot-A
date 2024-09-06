package org.firstinspires.ftc.teamcode.ftc7083.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.ftc7083.hardware.conversion.MotorConversion;

@Config
public class Motor implements DcMotorEx {
    // Defaults for GoBilda 5203 Series Yellow Jacket Planetary Gear Motor
    private static double DEFAULT_TICKS_PER_REV = 384.5;
    private static double DEFAULT_GEAR_RATIO = 1.0;

    private final DcMotorEx motorImpl;
    private final MotorConversion conv;

    /**
     * Instantiate a new motor for the robot.
     *
     * @param hardwareMap the mapping for all hardware on the robot
     * @param deviceName  the name of the motor as configured via the Driver Station
     */
    public Motor(HardwareMap hardwareMap, String deviceName) {
        this(hardwareMap, deviceName, DEFAULT_TICKS_PER_REV, DEFAULT_GEAR_RATIO);
    }

    /**
     * Instantiate a new motor for the robot.
     *
     * @param motorImpl   the motor as retrieved via the hardware map
     * @param ticksPerRev the number of ticks per rotation of the motor
     * @param gearRatio   the ratio of any external gears used with the motor
     */
    protected Motor(DcMotorEx motorImpl, double ticksPerRev, double gearRatio) {
        this.motorImpl = motorImpl;
        this.conv = new MotorConversion(ticksPerRev, gearRatio);
    }

    /**
     * Instantiate a new motor for the robot.
     *
     * @param hardwareMap the mapping for all hardware on the robot
     * @param deviceName  the name of the motor as configured via the Driver Station
     * @param ticksPerRev the number of ticks per rotation of the motor
     * @param gearRatio   the ratio of any external gears used with the motor
     */
    public Motor(HardwareMap hardwareMap, String deviceName, double ticksPerRev, double gearRatio) {
        this(hardwareMap.get(DcMotorEx.class, deviceName), ticksPerRev, gearRatio);
    }

    @Override
    public void setMotorEnable() {
        motorImpl.setMotorEnable();
    }

    @Override
    public void setMotorDisable() {
        motorImpl.setMotorDisable();
    }

    @Override
    public boolean isMotorEnabled() {
        return motorImpl.isMotorEnabled();
    }

    @Override
    public void setVelocity(double angularRate, AngleUnit unit) {
        motorImpl.setVelocity(angularRate, unit);
    }

    @Override
    public double getVelocity() {
        return motorImpl.getVelocity();
    }

    @Override
    public void setVelocity(double angularRate) {
        motorImpl.setVelocity(angularRate);
    }

    @Override
    public double getVelocity(AngleUnit unit) {
        return motorImpl.getVelocity(unit);
    }

    @Override
    @Deprecated
    public void setPIDCoefficients(RunMode mode, PIDCoefficients pidCoefficients) {
        motorImpl.setPIDCoefficients(mode, pidCoefficients);
    }

    @Override
    public void setPIDFCoefficients(RunMode mode, PIDFCoefficients pidfCoefficients) throws UnsupportedOperationException {
        motorImpl.setPIDFCoefficients(mode, pidfCoefficients);
    }

    @Override
    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
        motorImpl.setVelocityPIDFCoefficients(p, i, d, f);
    }

    @Override
    public void setPositionPIDFCoefficients(double p) {
        motorImpl.setPositionPIDFCoefficients(p);
    }

    @Override
    @Deprecated
    public PIDCoefficients getPIDCoefficients(RunMode mode) {
        return motorImpl.getPIDCoefficients(mode);
    }

    @Override
    public PIDFCoefficients getPIDFCoefficients(RunMode mode) {
        return motorImpl.getPIDFCoefficients(mode);
    }

    @Override
    public int getTargetPositionTolerance() {
        return motorImpl.getTargetPositionTolerance();
    }

    @Override
    public void setTargetPositionTolerance(int tolerance) {
        motorImpl.setTargetPosition(tolerance);
    }

    @Override
    public double getCurrent(CurrentUnit unit) {
        return motorImpl.getCurrent(unit);
    }

    @Override
    public double getCurrentAlert(CurrentUnit unit) {
        return motorImpl.getCurrentAlert(unit);
    }

    @Override
    public void setCurrentAlert(double current, CurrentUnit unit) {
        motorImpl.setCurrentAlert(current, unit);
    }

    @Override
    public boolean isOverCurrent() {
        return motorImpl.isOverCurrent();
    }

    @Override
    public MotorConfigurationType getMotorType() {
        return motorImpl.getMotorType();
    }

    @Override
    public void setMotorType(MotorConfigurationType motorType) {
        motorImpl.setMotorType(motorType);
    }

    @Override
    public DcMotorController getController() {
        return motorImpl.getController();
    }

    @Override
    public int getPortNumber() {
        return motorImpl.getPortNumber();
    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return getZeroPowerBehavior();
    }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        motorImpl.setZeroPowerBehavior(zeroPowerBehavior);
    }

    @Override
    @Deprecated
    public void setPowerFloat() {
        motorImpl.setPowerFloat();
    }

    @Override
    public boolean getPowerFloat() {
        return motorImpl.getPowerFloat();
    }

    /**
     * Sets the position of the motor to the specified number of degrees.
     *
     * @param degrees the degrees to which to set the motor
     */
    public void setDegrees(double degrees) {
        double position = conv.degreesToPosition(degrees);
        setTargetPosition((int) position);
    }

    /**
     * Gets the current degree offset of the motor.
     *
     * @return the current degree offset of the motor
     */
    public double getCurrentDegrees() {
        return conv.positionToDegrees(getCurrentPosition());
    }

    @Override
    public int getTargetPosition() {
        return motorImpl.getTargetPosition();
    }

    @Override
    public void setTargetPosition(int position) {
        motorImpl.setTargetPosition(position);
    }

    @Override
    public boolean isBusy() {
        return motorImpl.isBusy();
    }

    @Override
    public int getCurrentPosition() {
        return motorImpl.getCurrentPosition();
    }

    @Override
    public RunMode getMode() {
        return motorImpl.getMode();
    }

    @Override
    public void setMode(RunMode mode) {
        motorImpl.setMode(mode);
    }

    @Override
    public Direction getDirection() {
        return motorImpl.getDirection();
    }

    @Override
    public void setDirection(Direction direction) {
        motorImpl.setDirection(direction);
    }

    @Override
    public double getPower() {
        return motorImpl.getPower();
    }

    @Override
    public void setPower(double power) {
        motorImpl.setPower(power);
    }

    @Override
    public Manufacturer getManufacturer() {
        return motorImpl.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return motorImpl.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return motorImpl.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return motorImpl.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        motorImpl.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        motorImpl.close();
    }
}
