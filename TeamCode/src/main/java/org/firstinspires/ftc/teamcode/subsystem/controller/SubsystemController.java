package org.firstinspires.ftc.teamcode.subsystem.controller;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Controller is the base interface for all controllers on the robot. Controllers use the gamepad
 * inputs to manage their subsystems, translating the gamepad inputs (buttons, joysticks, etc.)
 * into actions taken by their managed subsystems.
 */
public interface SubsystemController {
    void execute(Gamepad gamepad1, Gamepad gamepad2);
}
