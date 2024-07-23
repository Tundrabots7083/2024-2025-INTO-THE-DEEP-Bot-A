package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.field.TeamElementLocation;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

/**
 * VisionProcessor determines the location of the team prop based on color saturation. It looks
 * at two spike marks, the `left` and `right`, to determine if the team prop is located on one of
 * those two spike marks. If so, it returns the location. If not, it infers the team prop must be
 * on the right spike mark and returns that location.
 */
@Config
public class VisionProcessor implements org.firstinspires.ftc.vision.VisionProcessor {
    public static int LEFT_RECTANGLE_X = 0;
    public static int LEFT_RECTANGLE_WIDTH = 155;
    public static int LEFT_RECTANGLE_Y = 365;
    public static int LEFT_RECTANGLE_HEIGHT = 105;
    public static int MIDDLE_RECTANGLE_X = 375;
    public static int MIDDLE_RECTANGLE_WIDTH = 250;
    public static int MIDDLE_RECTANGLE_Y = 310;
    public static int MIDDLE_RECTANGLE_HEIGHT = 95;

    public static double MIN_PERCENT_DIFFERENCE = 33;
    public static double MIN_LEFT_SAT = 50;
    public static double MIN_MIDDLE_SAT = 30;

    private final Telemetry telemetry;
    public Rect rectLeft = new Rect(LEFT_RECTANGLE_X, LEFT_RECTANGLE_Y, LEFT_RECTANGLE_WIDTH, LEFT_RECTANGLE_HEIGHT);
    public Rect rectMiddle = new Rect(MIDDLE_RECTANGLE_X, MIDDLE_RECTANGLE_Y, MIDDLE_RECTANGLE_WIDTH, MIDDLE_RECTANGLE_HEIGHT);

    private TeamElementLocation selection = TeamElementLocation.UNKNOWN;
    private Mat submat = new Mat();
    private final Mat hsvMat = new Mat();

    /**
     * Creates a new vision processor.
     *
     * @param telemetry the telemetry to be used for any output to the driver station.
     */
    public VisionProcessor(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    /**
     * Initializes the vision processor. This is a no-op for this class.
     *
     * @param width       width of the camera image.
     * @param height      height of the camera image.
     * @param calibration camera calibration information
     */
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    /**
     * Processes the frame from the webcam associated with this vision processor.
     *
     * @param frame            the frame from the webcam.
     * @param captureTimeNanos the time it took to capture the frame.
     * @return the location of the team prop on the frame.
     */
    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);

        double satRectLeft = getAvgSaturation(hsvMat, rectLeft);
        double satRectMiddle = getAvgSaturation(hsvMat, rectMiddle);

        telemetry.addData("[VISION] Left Spike", satRectLeft);
        telemetry.addData("[VISION] Middle Spike", satRectMiddle);

        double percentDifference = getPercentDifference(satRectLeft, satRectMiddle);
        telemetry.addData("[VISION] Percent Difference", percentDifference);

        telemetry.update();

        if (percentDifference < MIN_PERCENT_DIFFERENCE) {
            return TeamElementLocation.RIGHT_SPIKE_MARK;
        } else if (satRectLeft > satRectMiddle && satRectLeft > MIN_LEFT_SAT) {
            return TeamElementLocation.LEFT_SPIKE_MARK;
        } else if (satRectMiddle > satRectLeft && satRectMiddle > MIN_MIDDLE_SAT) {
            return TeamElementLocation.MIDDLE_SPIKE_MARK;
        }
        return TeamElementLocation.RIGHT_SPIKE_MARK;
    }

    /**
     * Determines the percentage difference between the two numbers.
     *
     * @param val1 the first number.
     * @param val2 the second number.
     * @return the percentage difference between the two numbers.
     */
    private double getPercentDifference(double val1, double val2) {
        return Math.abs(val1 - val2) / ((val1 + val2) / 2) * 100;
    }

    /**
     * Gets the average color saturation within the rectangle on the frame.
     *
     * @param input the frame from the webcam.
     * @param rect  the rectangle in which to check the color saturation.
     * @return the average color saturation within the rectangle.
     */
    protected double getAvgSaturation(Mat input, Rect rect) {
        submat = input.submat(rect);
        Scalar color = Core.mean(submat);
        return color.val[1];
    }

    /**
     * Draws the rectangle on the frame on the driver station.
     *
     * @param rect                 the rectangle to draw.
     * @param scaleBmpPxToCanvasPx the scale factor for the pixels.
     * @return the corresponding rectangle to draw on the driver station.
     */
    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }

    /**
     * Draws the frames on the android device. If the team prop is located within one of the
     * rectangles, the color for the rectangle is `red`; if the team prop is not within the
     * rectangle, the color for the rectangle is `green`.
     *
     * @param canvas               the "draw" cells
     * @param onscreenWidth        the width of the frame
     * @param onscreenHeight       the height of the frames
     * @param scaleBmpPxToCanvasPx scale factor for the number of pixels
     * @param scaleCanvasDensity   the density of the "draw" cells
     * @param userContext          the spike mark selection
     */
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint selectedPaint = new Paint();
        selectedPaint.setColor(Color.RED);
        selectedPaint.setStyle(Paint.Style.STROKE);
        selectedPaint.setStrokeWidth(scaleCanvasDensity * 4);

        Paint nonSelectedPaint = new Paint(selectedPaint);
        nonSelectedPaint.setColor(Color.GREEN);

        android.graphics.Rect drawRectangleLeft = makeGraphicsRect(rectLeft, scaleBmpPxToCanvasPx);
        android.graphics.Rect drawRectangleMiddle = makeGraphicsRect(rectMiddle, scaleBmpPxToCanvasPx);

        selection = (TeamElementLocation) userContext;
        telemetry.addData("Selection", selection);
        switch (selection) {
            case LEFT_SPIKE_MARK:
                canvas.drawRect(drawRectangleLeft, selectedPaint);
                canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);
                break;
            case MIDDLE_SPIKE_MARK:
                canvas.drawRect(drawRectangleLeft, nonSelectedPaint);
                canvas.drawRect(drawRectangleMiddle, selectedPaint);
                break;
            case RIGHT_SPIKE_MARK:
            case UNKNOWN:
                canvas.drawRect(drawRectangleLeft, nonSelectedPaint);
                canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);
                break;
        }
    }

    /**
     * Returns the location of the team prop.
     *
     * @return the location of the team prop.
     */
    public TeamElementLocation getSelection() {
        return selection;
    }
}
