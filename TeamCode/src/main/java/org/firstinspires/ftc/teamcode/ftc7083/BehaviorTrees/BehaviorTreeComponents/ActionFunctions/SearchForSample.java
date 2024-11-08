package org.firstinspires.ftc.teamcode.ftc7083.BehaviorTrees.BehaviorTreeComponents.ActionFunctions;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTrees.BehaviorTreeComponents.general.BlackBoardSingleton;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.ftc7083.feedback.PIDControllerImpl;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.MecanumDrive;

public class SearchForSample implements ActionFunction {
    MecanumDrive mecanumDrive;
    Telemetry telemetry;

    public static double KP = 0.01;
    public static double KI = 0.04;
    public static double KD = 0.0;
    public static double TOLERABLE_ERROR = 0.4; // degrees
    public static double INTAKE_ANGLE = 0.0;

    private final PIDControllerImpl pidController;

    protected Status lastStatus = Status.FAILURE;
    protected int runCount = 0;

    public SearchForSample(Telemetry telemetry, MecanumDrive mecanumDrive) {
        this.mecanumDrive = mecanumDrive;
        this.telemetry = telemetry;
        pidController = new PIDControllerImpl(KP, KI, KD);
        pidController.reset();
    }

    public Status perform(BlackBoardSingleton blackBoard) {
        Status status = Status.RUNNING;
        double turnPower;

        if (blackBoard.getValue("Tx") == null) {
            turnPower = 0.2;
        } else {
            turnPower = 0.0;
        }

        mecanumDrive.turn(turnPower);

        runCount++;
        telemetry.addData("[SearchForSample] status:", status);
        telemetry.update();

        return status;
    }
}
