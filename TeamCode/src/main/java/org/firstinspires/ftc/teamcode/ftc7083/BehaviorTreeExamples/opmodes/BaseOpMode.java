package org.firstinspires.ftc.teamcode.ftc7083.BehaviorTreeExamples.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.controls.Navigator;

/**
 * BaseOpMode is the base class for all opmodes on the system.
 */
public abstract class BaseOpMode extends OpMode {
    public final Robot robot = new Robot();
    public final Navigator nav = new Navigator(robot);

    @Override
    public void init() {
        robot.init(hardwareMap);
    }
}


