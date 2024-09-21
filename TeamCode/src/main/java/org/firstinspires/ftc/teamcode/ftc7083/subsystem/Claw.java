package org.firstinspires.ftc.teamcode.ftc7083.subsystem;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ftc7083.hardware.Servo;

/**
 * The Claw class implements a claw used by the robot to pick up samples
 * and specimens.  By default the claw is closed initially.  Users can request that
 * the claw be opened to some angle specified in degrees.  The claw is reset to its
 * initial closed position when a close is requested.
 */
@Config
public class Claw extends SubsystemBase {
    // Make default settable by FTC dashboard
    public static double DEFAULT_OPEN_DEGREES = 45.0;
    private final Servo servo;

    /**
     * Constructor
     *
     * @param hardwareMap  Servo configuration information
     * @param deviceName   Name of servo configuration information -- must match name configured
     *                     in Control Hub or Expansion Hub
     * @param maxDegrees   Maximum number of degrees that the claw servo can be opened to.
     */
    public Claw(HardwareMap hardwareMap, String deviceName, double maxDegrees) {
        servo = new Servo(hardwareMap, deviceName);
        servo.setDegrees(maxDegrees);
        // todo: May want to set initial position and direction for servo.
    }

    /**
     * Opens claw to a default angle of 45 degrees.
     *
     * @return  double:  The degrees the claw was opened to.
     */
    public double open() {
        servo.setDegrees(DEFAULT_OPEN_DEGREES);
        return servo.getDegrees();
    }

    /**
     * Opens claw to an angle of the requested degrees
     *
     * @param degrees  Size of the angle the claw is to be opened to.
     * @return  double:  The angle in degrees that the claw was opened to.
     */
    public double open(double degrees) {
        servo.setDegrees(degrees);
        return servo.getDegrees();
    }

    /**
     * Totally close the claw back to 0 degrees.
     *
     * @return double:  angle in degrees that claw was set to
     */
    public double close() {
        servo.setDegrees(0);
        return servo.getDegrees();
    }



}