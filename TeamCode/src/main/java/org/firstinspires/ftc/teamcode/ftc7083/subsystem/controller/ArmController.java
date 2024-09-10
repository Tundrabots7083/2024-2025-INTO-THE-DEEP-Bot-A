package org.firstinspires.ftc.teamcode.ftc7083.subsystem.controller;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Arm;

/**
 * Uses the gamepad controller to set the position of the arm and container servo.
 */
public class ArmController implements SubsystemController {
    private final Arm arm;
    private final Telemetry telemetry;

    /**
     * Initializes the arm hardware.
     *
     * @param hardwareMap the hardware map for the robot.
     */
    public ArmController(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        arm = new Arm(hardwareMap, telemetry);
        arm.setTarget(Arm.Position.Intake);
        arm.update();
    }

    /**
     * Sets the position of the arm, and calls the arm to update the position.
     *
     * @param gamepad1 Gamepad1
     * @param gamepad2 Gamepad2
     */
    @Override
    public void execute(Gamepad gamepad1, Gamepad gamepad2) {
        // Automatic update of controls
        if (gamepad1.dpad_down) {
            arm.setTarget(Arm.Position.Intake);
        } else if (gamepad1.dpad_left) {
            arm.setTarget(Arm.Position.ScoreLow);
        } else if (gamepad1.dpad_right) {
            arm.setTarget(Arm.Position.ScoreMedium);
        } else if (gamepad1.dpad_up) {
            arm.setTarget(Arm.Position.ScoreHigh);
        } else if (gamepad1.x) {
            arm.setTarget(Arm.Position.LaunchDrone);
        } else if (gamepad1.y) {
            arm.setTarget(Arm.Position.Hang);
        }

        arm.update();
    }
}

