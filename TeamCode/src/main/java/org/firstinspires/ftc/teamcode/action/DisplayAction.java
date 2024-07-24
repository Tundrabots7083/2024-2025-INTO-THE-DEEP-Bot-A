package org.firstinspires.ftc.teamcode.action;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.jetbrains.annotations.NotNull;

/**
 * Sends the text to the telemetry (doesn't seem to do anything right now)
 */
public class DisplayAction implements Action {
    private final String message;

    /**
     * Instantiates a display action that will send a message to the driver station.
     *
     * @param message the message to send
     */
    public DisplayAction(String message) {
        this.message = message;
    }

    /**
     * Sends the message to the driver station.
     *
     * @param telemetryPacket the telemetry to use when displaying the message.
     * @return <code>false</code>, which causes the action to terminate.
     */
    @Override
    public boolean run(@NotNull TelemetryPacket telemetryPacket) {
        telemetryPacket.addLine(message);
        return false;
    }

    /**
     * Gets a string representation of this action.
     *
     * @return a string representation of this action
     */
    @NonNull
    @Override
    public String toString() {
        return "DisplayAction{" +
                "message='" + message + '\'' +
                '}';
    }
}
