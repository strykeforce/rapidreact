package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DriveAutonCommand;
import frc.robot.subsystems.DriveSubsystem;

public class TwoPathCommandGroup extends SequentialCommandGroup {
  public TwoPathCommandGroup(DriveSubsystem driveSubsystem, String path1, String path2) {
    addCommands(
        new DriveAutonCommand(driveSubsystem, path1), new DriveAutonCommand(driveSubsystem, path2));
  }
}