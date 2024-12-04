package org.firstinspires.ftc.teamcode.ftc7083.test;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS.Pose2D;

import org.firstinspires.ftc.teamcode.ftc7083.filter.Pose2DMovingAverageFilter;
import org.junit.Test;

public class MovingAverageFilterBaseTest {

    @SuppressLint("DefaultLocale")
    @Test
    public void filter() {
        Pose2DMovingAverageFilter filter = new Pose2DMovingAverageFilter(5, 11);
        for (int i = 0; i < 20; i++) {
            filter.filter(new Pose2D(1,1,Math.toRadians(1)));
        }
        Pose2D pose = filter.filter(new Pose2D(5,10,Math.toRadians(4)));
        assert String.format("%.5f", pose.x).equals(String.format("%.5f", (10.0 + 5) / 11));
        assert String.format("%.5f", pose.y).equals(String.format("%.5f", (10.0 + 10) / 11));
    }

    @SuppressLint("DefaultLocale")
    @Test
    public void removeMeasurement() {
        Pose2DMovingAverageFilter filter = new Pose2DMovingAverageFilter(5, 11);
        for (int i = 0; i < 20; i++) {
            filter.filter(new Pose2D(1,1,Math.toRadians(1)));
        }
        Pose2D pose = filter.removeMeasurement();
        assert pose.x == 1.0;
        assert pose.y == 1.0;
        assert String.format("%.5f", pose.h).equals(String.format("%.5f", Math.toRadians(1.0)));
    }

    @SuppressLint("DefaultLocale")
    @Test
    public void getAverage() {
        Pose2DMovingAverageFilter filter = new Pose2DMovingAverageFilter(5, 11);
        for (int i = 0; i < 20; i++) {
            filter.filter(new Pose2D(1,1,Math.toRadians(1)));
        }
        Pose2D pose = filter.getMean();
        assert pose.x == 1.0;
        assert pose.y == 1.0;
        assert String.format("%.5f", pose.h).equals(String.format("%.5f", Math.toRadians(1.0)));
    }

    @Test
    public void numMeasurements() {
        Pose2DMovingAverageFilter filter = new Pose2DMovingAverageFilter(5, 11);
        for (int i = 0; i < 4; i++) {
            filter.filter(new Pose2D(1,1,1));
        }
        assert filter.numMeasurements() == 4;
        filter.filter(new Pose2D(1,1,1));
        assert filter.numMeasurements() == 5;
        for (int i = 0; i < 20; i++) {
            filter.filter(new Pose2D(1,1,1));
        }
        assert filter.numMeasurements() == 11;
        filter.removeMeasurement();
        assert filter.numMeasurements() == 10;
    }

    @Test
    public void hasAverage() {
        Pose2DMovingAverageFilter filter = new Pose2DMovingAverageFilter(5, 11);
        for (int i = 0; i < 4; i++) {
            filter.filter(new Pose2D(1,1,1));
        }
        assert !filter.hasMean();
        filter.filter(new Pose2D(1,1,1));
        assert filter.hasMean();
    }

    @Test
    public void clear() {
        Pose2DMovingAverageFilter filter = new Pose2DMovingAverageFilter(5, 11);
        for (int i = 0; i < 5; i++) {
            double x = i+1;
            double y = i+2;
            double h = Math.toRadians(i+3);
            Pose2D measurement = new Pose2D(x, y, h);
            filter.filter(measurement);
        }
        filter.clear();
        assert filter.numMeasurements() == 0;
    }
}
