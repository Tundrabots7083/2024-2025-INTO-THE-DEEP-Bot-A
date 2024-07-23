package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.Subsystem;

/**
 * Extended interface for the FTCLib subsystem. This extends the FTCLib subsystem so that includes some
 *  * methods used by multiple subsystems.
 */
public interface SubsystemEx extends Subsystem {
    /**
     * Method to be called each time through the FTC OpMode loop, which allows the subsystem to
     * take any actions that it needs to fulfill its purpose. By default, this method does not
     * perform any action. Classes that need to perform actions should override it.
     */
    default void execute() {}
}
