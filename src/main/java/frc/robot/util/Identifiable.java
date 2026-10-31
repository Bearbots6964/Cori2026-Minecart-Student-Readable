package frc.robot.util;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

// FILE GUIDE: Shared interface for subsystems supporting WPILib SysId characterization commands.
public interface Identifiable {

    // STUDENT MAP: SYSID-CAPABLE SUBSYSTEM CONTRACT
    // PURPOSE: Lists the characterization commands a subsystem must supply for identifying motor behavior.
    // LOOK FOR: An interface lists required methods; it does not operate a motor by itself.
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction);

    // READ THIS: Provide a dynamic motor-characterization command, implemented by each subsystem.
    public Command sysIdDynamic(SysIdRoutine.Direction direction);

}
