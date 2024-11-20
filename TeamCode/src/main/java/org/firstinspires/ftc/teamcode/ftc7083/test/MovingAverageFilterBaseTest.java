package org.firstinspires.ftc.teamcode.ftc7083.test;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.ftc7083.filter.Pose2dMovingAverageFilter;
import org.junit.Test;

public class MovingAverageFilterBaseTest {

    @SuppressLint("DefaultLocale")
    @Test
    public void filter() {
        Pose2dMovingAverageFilter filter = new Pose2dMovingAverageFilter(5, 11);
        for (int i = 0; i < 20; i++) {
            filter.filter(new Pose2d(1,1,Math.toRadians(1)));
        }
        Pose2d pose = filter.filter(new Pose2d(5,10,Math.toRadians(4)));
        assert String.format("%.5f", pose.position.x).equals(String.format("%.5f", (10.0 + 5) / 11));
        assert String.format("%.5f", pose.position.y).equals(String.format("%.5f", (10.0 + 10) / 11));
    }

    @SuppressLint("DefaultLocale")
    @Test
    public void removeMeasurement() {
        Pose2dMovingAverageFilter filter = new Pose2dMovingAverageFilter(5, 11);
        for (int i = 0; i < 20; i++) {
            filter.filter(new Pose2d(1,1,Math.toRadians(1)));
        }
        Pose2d pose = filter.removeMeasurement();
        assert pose.position.x == 1.0;
        assert pose.position.y == 1.0;
        assert String.format("%.5f", pose.heading.toDouble()).equals(String.format("%.5f", Math.toRadians(1.0)));
    }

    @SuppressLint("DefaultLocale")
    @Test
    public void getAverage() {
        Pose2dMovingAverageFilter filter = new Pose2dMovingAverageFilter(5, 11);
        for (int i = 0; i < 20; i++) {
            filter.filter(new Pose2d(1,1,Math.toRadians(1)));
        }
        Pose2d pose = filter.getAverage();
        assert pose.position.x == 1.0;
        assert pose.position.y == 1.0;
        assert String.format("%.5f", pose.heading.toDouble()).equals(String.format("%.5f", Math.toRadians(1.0)));
    }

    @Test
    public void numMeasurements() {
        Pose2dMovingAverageFilter filter = new Pose2dMovingAverageFilter(5, 11);
        for (int i = 0; i < 4; i++) {
            filter.filter(new Pose2d(1,1,1));
        }
        assert filter.numMeasurements() == 4;
        filter.filter(new Pose2d(1,1,1));
        assert filter.numMeasurements() == 5;
        for (int i = 0; i < 20; i++) {
            filter.filter(new Pose2d(1,1,1));
        }
        assert filter.numMeasurements() == 11;
        filter.removeMeasurement();
        assert filter.numMeasurements() == 10;
    }

    @Test
    public void hasAverage() {
        Pose2dMovingAverageFilter filter = new Pose2dMovingAverageFilter(5, 11);
        for (int i = 0; i < 4; i++) {
            filter.filter(new Pose2d(1,1,1));
        }
        assert !filter.hasAverage();
        filter.filter(new Pose2d(1,1,1));
        assert filter.hasAverage();
    }

    @Test
    public void clear() {
        Pose2dMovingAverageFilter filter = new Pose2dMovingAverageFilter(5, 11);
        for (int i = 0; i < 5; i++) {
            double x = i+1;
            double y = i+2;
            double h = Math.toRadians(i+3);
            Pose2d measurement = new Pose2d(x, y, h);
            filter.filter(measurement);
        }
        filter.clear();
        assert filter.numMeasurements() == 0;
    }
}
