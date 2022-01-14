package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;

public class ZeroGyroCommand extends InstantCommand {
    DriveSubsystem driveSubsystem;
    public ZeroGyroCommand (DriveSubsystem driveSubsystem)

    {
        addRequirements(driveSubsystem);

        this.driveSubsystem = driveSubsystem;
    }

    @Override
    public void initialize() {
        driveSubsystem.resetGyro();
    }
    
}
