package org.firstinspires.ftc.teamcode.ftc7083;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Arm;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Claw;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.IntakeAndScoringSubsystem;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.LinearSlide;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Wrist;

public class Robot {
    public static final Telemetry telemetry = new Telemetry();
    public static final Claw claw = new Claw(null, telemetry);
    public static final Arm arm = new Arm(null, telemetry);
    public static final LinearSlide linearSlide = new LinearSlide(null, telemetry);
    public static final Wrist wrist = new Wrist(null, telemetry);
    public static final IntakeAndScoringSubsystem intakeAndScoringSubsystem = new IntakeAndScoringSubsystem(null, telemetry);

    public static Robot getInstance() {
        return new Robot();
    }
}
