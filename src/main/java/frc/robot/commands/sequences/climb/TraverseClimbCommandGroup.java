package frc.robot.commands.sequences.climb;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.climb.AutoTraverseClimbCommand;
import frc.robot.commands.climb.InitiateClimbCommand;
import frc.robot.commands.drive.DriveClimbCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class TraverseClimbCommandGroup extends SequentialCommandGroup {
  public TraverseClimbCommandGroup(
      ClimbSubsystem climbSubsystem, DriveSubsystem driveSubsystem, Joystick driveJoystick) {
    addCommands(
        new ParallelCommandGroup(
            new InitiateClimbCommand(climbSubsystem),
            new DriveClimbCommand(driveSubsystem, driveJoystick, climbSubsystem)),
        new AutoTraverseClimbCommand(climbSubsystem));
  }
}
