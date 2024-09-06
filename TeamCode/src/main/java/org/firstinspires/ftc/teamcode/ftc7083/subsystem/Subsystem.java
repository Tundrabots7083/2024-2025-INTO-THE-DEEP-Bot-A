package org.firstinspires.ftc.teamcode.ftc7083.subsystem;

/**
 * Interface for an FTC subsystem
 */
public interface Subsystem {
    /**
     * Method to be called each time through the FTC OpMode loop, which allows the subsystem to
     * take any actions that it needs to fulfill its purpose. By default, this method does not
     * perform any action. Classes that need to perform actions should override it.
     */
    default void execute() {
    }
}
