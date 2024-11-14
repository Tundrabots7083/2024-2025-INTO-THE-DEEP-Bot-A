package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.Robot;
import org.firstinspires.ftc.teamcode.ftc7083.autonomous.trajectory.BlueBasket;
import org.firstinspires.ftc.teamcode.ftc7083.autonomous.trajectory.BlueChamber;

public class MeepMeepTesting {
    public static void main(String[] args) {
        Robot.init(null, new Telemetry());
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(75, 75, Math.toRadians(180), Math.toRadians(180), 18)
                .build();

        // Trajectory builders for Blue and Red alliances when scoring on the chambers or in the baskets
//        BlueChamber trajectoryBuilder = new BlueChamber(myBot.getDrive().actionBuilder(new Pose2d(BlueChamber.INITIAL_POSE_X, BlueChamber.INITIAL_POSE_Y, BlueChamber.INITIAL_HEADING)));
        BlueBasket trajectoryBuilder = new BlueBasket(myBot.getDrive().actionBuilder(new Pose2d(BlueBasket.INITIAL_POSE_X, BlueBasket.INITIAL_POSE_Y, BlueBasket.INITIAL_HEADING)));

        myBot.runAction(trajectoryBuilder.getTrajectory());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
