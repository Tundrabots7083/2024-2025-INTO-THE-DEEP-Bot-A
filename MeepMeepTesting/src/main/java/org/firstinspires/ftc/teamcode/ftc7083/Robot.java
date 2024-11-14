package org.firstinspires.ftc.teamcode.ftc7083;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Arm;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Claw;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.IntakeAndScoringSubsystem;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.LinearSlide;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Wrist;

public class Robot {
    private static Robot robot = null;
    public final Claw claw;
    public final Arm arm;
    public final LinearSlide linearSlide;
    public final Wrist wrist;
    public final IntakeAndScoringSubsystem intakeAndScoringSubsystem;

    public Robot(HardwareMap hardwareMap, @NonNull Telemetry telemetry, OpModeType opModeType) {
        robot = this;

        claw = new Claw(hardwareMap, telemetry);
        arm = new Arm(hardwareMap, telemetry);
        linearSlide = new LinearSlide(hardwareMap, telemetry);
        wrist = new Wrist(hardwareMap, telemetry);
        intakeAndScoringSubsystem = new IntakeAndScoringSubsystem(hardwareMap, telemetry);
    }

    /**
     * Initializes the hardware mechanisms for the robot. This creates the singleton that is retrieved
     * using the <code>getInstance</code> method.
     *
     * @param hardwareMap hardware map for the robot.
     * @param telemetry   telemetry class for displaying data.
     * @return the robot instance
     */
    public static Robot init(HardwareMap hardwareMap, @NonNull Telemetry telemetry) {
        return init(hardwareMap, telemetry, OpModeType.TELEOP);
    }

    /**
     * Initializes the hardware mechanisms for the robot. This creates the singleton that is retrieved
     * using the <code>getInstance</code> method.
     *
     * @param hardwareMap hardware map for the robot.
     * @param telemetry   telemetry class for displaying data.
     * @param opModeType  the type of opmode the robot is being used for
     * @return the robot instance
     */
    public static Robot init(HardwareMap hardwareMap, @NonNull Telemetry telemetry, OpModeType opModeType) {
        robot = new Robot(hardwareMap, telemetry, opModeType);
        return robot;
    }

    /**
     * Gets the singleton instance of the robot.
     */
    public static Robot getInstance() {
        return robot;
    }

    // Enum to specify opmode type
    public enum OpModeType {
        /**
         * Driver controlled OpMode
         */
        TELEOP,
        /**
         * Autonomous OpMode
         */
        AUTO
    }
}
