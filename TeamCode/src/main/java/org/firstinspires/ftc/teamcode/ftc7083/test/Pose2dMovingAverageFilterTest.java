package org.firstinspires.ftc.teamcode.ftc7083.test;

import com.acmerobotics.roadrunner.Pose2d;

import org.junit.Test;
import org.firstinspires.ftc.teamcode.ftc7083.filter.Pose2dMovingAverageFilter;

public class Pose2dMovingAverageFilterTest {

    @Test
    public void startingValue() {
        Pose2dMovingAverageFilter filter = new Pose2dMovingAverageFilter(5, 11);
        Pose2d pose = filter.startingValue();
        assert pose.position.x == 0.0;
        assert pose.position.y == 0.0;
        assert pose.heading.toDouble() == 0.0;
    }

    @Test
    public void addValue() {
        Pose2dMovingAverageFilter filter = new Pose2dMovingAverageFilter(5, 11);
        Pose2d sum = new Pose2d(1, 2, Math.toRadians(3));
        Pose2d measurement = new Pose2d(5, 10, Math.toRadians(15));
        Pose2d pose = filter.addValue(sum, measurement);
        assert pose.position.x == 6.0;
        assert pose.position.y == 12.0;
        assert Math.toDegrees(pose.heading.toDouble()) == 18.0;
    }

    @Test
    public void remValue() {
        Pose2dMovingAverageFilter filter = new Pose2dMovingAverageFilter(5, 11);
        Pose2d sum = new Pose2d(5, 10, Math.toRadians(15));
        Pose2d measurement = new Pose2d(1, 2, Math.toRadians(3));
        Pose2d pose = filter.remValue(sum, measurement);
        assert pose.position.x == 4.0;
        assert pose.position.y == 8.0;
        assert pose.heading.toDouble() == Math.toRadians(15) - Math.toRadians(3);
    }

    @Test
    public void getAverage() {
        Pose2dMovingAverageFilter filter = new Pose2dMovingAverageFilter(5, 11);
        double totalX = 0;
        double totalY = 0;
        double totalH = 0;
        for (int i = 0; i < 5; i++) {
            double x = i+1;
            double y = i+2;
            double h = Math.toRadians(i+3);
            Pose2d measurement = new Pose2d(x, y, h);
            filter.filter(measurement);
            totalX += x;
            totalY += y;
            totalH += h;
        }
        double avgX = totalX / 5.0;
        double avgY = totalY / 5.0;
        double avgH = totalH / 5.0;
        Pose2d pose = filter.getAverage();
        assert pose.position.x == avgX;
        assert pose.position.y == avgY;
        assert pose.heading.toDouble() == avgH;
    }
}
