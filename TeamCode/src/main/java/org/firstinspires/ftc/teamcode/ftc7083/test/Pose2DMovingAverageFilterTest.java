package org.firstinspires.ftc.teamcode.ftc7083.test;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS.Pose2D;

import org.junit.Test;
import org.firstinspires.ftc.teamcode.ftc7083.filter.Pose2DMovingAverageFilter;

public class Pose2DMovingAverageFilterTest {

    @Test
    public void startingValue() {
        Pose2DMovingAverageFilter filter = new Pose2DMovingAverageFilter(5, 11);
        Pose2D pose = filter.startingValue();
        assert pose.x == 0.0;
        assert pose.y == 0.0;
        assert pose.h == 0.0;
    }

    @Test
    public void addValue() {
        Pose2DMovingAverageFilter filter = new Pose2DMovingAverageFilter(5, 11);
        Pose2D sum = new Pose2D(1, 2, Math.toRadians(3));
        Pose2D measurement = new Pose2D(5, 10, Math.toRadians(15));
        Pose2D pose = filter.addValue(sum, measurement);
        assert pose.x == 6.0;
        assert pose.y == 12.0;
        assert Math.toDegrees(pose.h) == 18.0;
    }

    @Test
    public void remValue() {
        Pose2DMovingAverageFilter filter = new Pose2DMovingAverageFilter(5, 11);
        Pose2D sum = new Pose2D(5, 10, Math.toRadians(15));
        Pose2D measurement = new Pose2D(1, 2, Math.toRadians(3));
        Pose2D pose = filter.remValue(sum, measurement);
        assert pose.x == 4.0;
        assert pose.y == 8.0;
        assert pose.h == Math.toRadians(15) - Math.toRadians(3);
    }

    @Test
    public void getAverage() {
        Pose2DMovingAverageFilter filter = new Pose2DMovingAverageFilter(5, 11);
        double totalX = 0;
        double totalY = 0;
        double totalH = 0;
        for (int i = 0; i < 5; i++) {
            double x = i+1;
            double y = i+2;
            double h = Math.toRadians(i+3);
            Pose2D measurement = new Pose2D(x, y, h);
            filter.filter(measurement);
            totalX += x;
            totalY += y;
            totalH += h;
        }
        double avgX = totalX / 5.0;
        double avgY = totalY / 5.0;
        double avgH = totalH / 5.0;
        Pose2D pose = filter.getMean();
        assert pose.x == avgX;
        assert pose.y == avgY;
        assert pose.h == avgH;
    }
}
