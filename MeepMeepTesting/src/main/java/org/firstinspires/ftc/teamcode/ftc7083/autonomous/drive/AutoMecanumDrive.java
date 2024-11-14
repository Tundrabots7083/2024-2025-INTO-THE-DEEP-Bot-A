package org.firstinspires.ftc.teamcode.ftc7083.autonomous.drive;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

/**
 * Wrapper class to provide an actionBuilder interface for the AutoMecanumDrive class. This is
 * provided to match what is used for "normal" autonomous mode operation.
 */
public class AutoMecanumDrive {
    private final RoadRunnerBotEntity bot;

    /**
     * Instantiates the AutoMecanumDrive wrapper class.
     *
     * @param bot the RoadRunner bot to be wrapped.
     */
    public AutoMecanumDrive(RoadRunnerBotEntity bot) {
        this.bot = bot;
    }

    /**
     * Gets a TrajectoryActionBuilder from the RoadRunner bot and returns it.
     *
     * @param pose the initial pose for the trajectory action builder.
     * @return the trajectory action builder from the RoadRunner bot.
     */
    public TrajectoryActionBuilder actionBuilder(Pose2d pose) {
        return bot.getDrive().actionBuilder(pose);
    }
}
