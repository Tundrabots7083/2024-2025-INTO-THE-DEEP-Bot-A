package org.firstinspires.ftc.teamcode.ftc7083.subsystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;
import org.firstinspires.ftc.teamcode.ftc7083.field.TeamElementLocation;
import org.firstinspires.ftc.teamcode.ftc7083.vision.VisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

/**
 * A vision sensor used to create and obtain data from a vision processor and determine the
 * location of the team prop.
 */
public class Webcam extends SubsystemBase {

    private final VisionProcessor visionProcessor;
    private VisionPortal webcamPortal;

    /**
     * Creates a new vision sensor for the webcam with the given name.
     *
     * @param hardwareMap mapping of all the hardware on the robot
     * @param telemetry   the telemetry used to provide output on the driver station.
     * @param deviceName  name of the webcam
     */
    public Webcam(@NonNull HardwareMap hardwareMap, @NonNull Telemetry telemetry, @NonNull String deviceName) {

        WebcamName webcamName = hardwareMap.get(WebcamName.class, deviceName);

        visionProcessor = new VisionProcessor(telemetry);
        webcamPortal = VisionPortal.easyCreateWithDefaults(webcamName, visionProcessor);
        FtcDashboard.getInstance().startCameraStream(webcamPortal, 0);
        CameraStreamServer.getInstance().setSource(webcamPortal);
    }

    /**
     * Returns indication as to whether the webcam portal used by the vision sensor is or is not
     * initialized.
     *
     * @return <code>true</code> if the vision sensor is initialized;
     * <code>false</code> if it is not.
     */
    public boolean webcamInitialized() {
        return webcamPortal != null && webcamPortal.getCameraState() == VisionPortal.CameraState.STREAMING;
    }

    /**
     * Returns the location of the team element.
     *
     * @return the location of the team element.
     */
    public TeamElementLocation getTeamElementLocation() {
        return visionProcessor.getSelection();
    }

    /**
     * Closes the webcam portal used by the vision sensor. The vision sensor should not be used
     * once the webcam portal has been closed.
     */
    public void close() {
        if (webcamPortal != null && webcamPortal.getCameraState() != VisionPortal.CameraState.CAMERA_DEVICE_CLOSED) {
            FtcDashboard.getInstance().stopCameraStream();
            webcamPortal.close();
            webcamPortal = null;
        }
    }

    /**
     * Gets a string representation of the webcam.
     *
     * @return a string representation of the webcam
     */
    @NonNull
    @Override
    public String toString() {
        return "Webcam{" +
                "visionProcessor=" + visionProcessor +
                ", webcamPortal=" + webcamPortal +
                '}';
    }
}
