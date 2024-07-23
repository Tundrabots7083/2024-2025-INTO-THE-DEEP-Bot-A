package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;

/**
 * An extended version of the FTCLib CRServo class that includes pulse width modulation (PWM)
 * capabilities.
 * <p>
 * For FTC, servo ports are managed as a pair. Disabling PWM for one port will
 * disable it for the other as well, and enabling PWM for one port enables it for both. Be
 * careful when using this class for your CR servo. If you do not want nor need PWM management
 * for the servo port pair, use <code>CRServo</code>provided by FTCLib.
 */
public class CRServoEx extends CRServo implements PwmControl {
    /**
     * The constructor for the CR Servo.
     *
     * @param hardwareMap the mapping of all hardware on the robot
     * @param id          the device name for the servo
     */
    public CRServoEx(HardwareMap hardwareMap, String id) {
        super(hardwareMap, id);

        // Override the CRServo implementation in the superclass with one with PWM management support
        // Since CRServoImplEx extends CRServoImpl this operation is safe to perform.
        crServo = hardwareMap.get(CRServoImplEx.class, id);
    }

    /**
     * Gets the PWM range limits for the servo
     *
     * @return the PWM range limits for the servo
     */
    @Override
    public PwmRange getPwmRange() {
        CRServoImplEx crServoEx = (CRServoImplEx) crServo;
        return crServoEx.getPwmRange();
    }

    /**
     * Sets the PWM range limits for the servo.
     *
     * @param range the new PWM range limits for the servo
     */
    @Override
    public void setPwmRange(PwmRange range) {
        CRServoImplEx crServoEx = (CRServoImplEx) crServo;
        crServoEx.setPwmRange(range);
    }

    /**
     * Enables PWM for the servo.
     */
    @Override
    public void setPwmEnable() {
        CRServoImplEx crServoEx = (CRServoImplEx) crServo;
        crServoEx.setPwmEnable();
    }

    /**
     * Disables PWM for the servo.
     */
    @Override
    public void setPwmDisable() {
        CRServoImplEx crServoEx = (CRServoImplEx) crServo;
        crServoEx.setPwmDisable();
    }

    /**
     * Gets indication as to whether PWM is enabled for the servo.
     *
     * @return <code>true</code> if PWM is enabled; <code>false</code> if PWM is not enabled.
     */
    @Override
    public boolean isPwmEnabled() {
        CRServoImplEx crServoEx = (CRServoImplEx) crServo;
        return crServoEx.isPwmEnabled();
    }

    /**
     * Sets the power level of the motor, expressed as a fraction of the maximum possible power / speed
     * supported according to the run mode in which the servo is operating.
     * <p>
     * Setting a power level of zero will brake the servo.
     *
     * @param output the new power level of the servo, a value in the interval [-1.0, 1.0]
     */
    @Override
    public void set(double output) {
        crServo.setPower(output);
        if (output == 0) {
            setPwmDisable();
        }
    }
}
