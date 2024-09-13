package org.firstinspires.ftc.teamcode.ftc7083.hardware;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.math.BigDecimal;
import java.math.MathContext;

public class Motor implements DcMotorEx {
    private final DcMotorEx motorImpl;
    private final Telemetry telemetry;
    private BigDecimal ticksPerInch;
    private BigDecimal ticksPerDegree;

    /**
     * Instantiate a new motor for the robot.
     *
     * @param hardwareMap the mapping for all hardware on the robot
     * @param deviceName  the name of the motor as configured via the Driver Station
     */
    public Motor(HardwareMap hardwareMap, Telemetry telemetry, String deviceName) {
        this(hardwareMap.get(DcMotorEx.class, deviceName), telemetry);
    }

    /**
     * Instantiate a new motor for the robot.
     *
     * @param motorImpl the motor as retrieved via the hardware map
     */
    protected Motor(DcMotorEx motorImpl, Telemetry telemetry) {
        this.motorImpl = motorImpl;
        this.telemetry = telemetry;
    }

    /**
     * Sets the number of motor ticks required to move what is attached to the motor one inch.
     *
     * @param ticksPerRev  number of motor ticks to rotate the motor 360 degrees
     * @param inchesPerRev the number of inches moved per revolution of the motor
     * @return this motor
     */
    public Motor setTicksPerInch(double ticksPerRev, double inchesPerRev) {
        return setTicksPerInch(ticksPerRev * inchesPerRev);
    }

    /**
     * Gets the number of motor ticks required to move what is attached to the motor one inch.
     *
     * @return the number of motor ticks required to move what is attached to the motor one inch
     */
    public double getTicksPerInch() {
        if (ticksPerInch == null) {
            return 0;
        }
        return ticksPerInch.doubleValue();
    }

    /**
     * Sets the number of motor ticks required to move what is attached to the motor one inch.
     *
     * @param ticksPerInch number of motor ticks to move one inch.
     * @return this motor
     */
    public Motor setTicksPerInch(double ticksPerInch) {
        this.ticksPerInch = new BigDecimal(ticksPerInch);
        return this;
    }

    /**
     * Gets the number of motor ticks required to move what is attached to the motor one degree
     * of rotation.
     *
     * @return number of motor ticks to move one degree of rotation
     */
    public double getTicksPerDegree() {
        if (ticksPerDegree == null) {
            return 0;
        }
        return ticksPerDegree.doubleValue();
    }

    /**
     * Sets the number of motor ticks required to move what is attached to the motor one degree
     * of rotation.
     *
     * @param ticksPerDegree number of motor ticks to move one degree of rotation
     * @return this motor
     */
    public Motor setTicksPerDegree(double ticksPerDegree) {
        this.ticksPerDegree = new BigDecimal(ticksPerDegree);
        return this;
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
     * <p>
     * Before using, make sure you call <code>setTicksPerDegree</code> to set the number of ticks
     * required to move whatever is attached to the motor one degree.
     *
     * @param degrees the degrees to which to set the motor
     */
    public void setDegrees(double degrees) {
        BigDecimal deg = new BigDecimal(degrees);
        BigDecimal position = deg.multiply(ticksPerDegree);
        setTargetPosition(position.intValue());
    }

    /**
     * Gets the current degree offset of the motor.
     * <p>
     * Before using, make sure you call <code>setTicksPerDegree</code> to set the number of ticks
     * required to move whatever is attached to the motor one degree.
     *
     * @return the current degree offset of the motor
     */
    public double getCurrentDegrees() {
        BigDecimal pos = new BigDecimal(getCurrentPosition());
        BigDecimal degrees = pos.divide(ticksPerDegree, MathContext.DECIMAL128);
        return degrees.doubleValue();
    }

    /**
     * Gets the current inches the motor has moved from the zero position.
     *
     * <p>
     * Before using, make sure you call <code>setTicksPerInches</code> to set the number of ticks
     * required to move whatever is attached to the motor one inch.
     *
     * @return the current inches the motor has moved from the zero position
     */
    public double getInches() {
        BigDecimal pos = new BigDecimal(getCurrentPosition());
        BigDecimal inches = pos.divide(ticksPerInch, MathContext.DECIMAL128);
        return inches.doubleValue();
    }

    /**
     * Sets the position of the motor to the specified number of inches.
     * <p>
     * Before using, make sure you call <code>setTicksPerInches</code> to set the number of ticks
     * required to move whatever is attached to the motor one inch.
     *
     * @param inches the inches to which to set the motor
     */
    public void setInches(double inches) {
        BigDecimal in = new BigDecimal(inches);
        BigDecimal position = in.multiply(ticksPerInch);
        setTargetPosition(position.intValue());
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
