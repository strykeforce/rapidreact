package frc.robot.commands.sequences.climb;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.climb.AutoMidClimbCommand;
import frc.robot.commands.climb.InitiateClimbCommand;
import frc.robot.commands.drive.DriveClimbCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class MidClimbCommandGroup extends SequentialCommandGroup {
  public MidClimbCommandGroup(
      ClimbSubsystem climbSubsystem,
      DriveSubsystem driveSubsystem,
      Joystick driveJoystick,
      TurretSubsystem turretSubsystem) {
    addCommands(
        new ParallelCommandGroup(
            new InitiateClimbCommand(climbSubsystem, turretSubsystem),
            new DriveClimbCommand(driveSubsystem, driveJoystick, climbSubsystem)),
        new AutoMidClimbCommand(climbSubsystem));
  }
}
