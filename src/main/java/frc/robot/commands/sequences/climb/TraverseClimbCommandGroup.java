package frc.robot.commands.sequences.climb;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.climb.AutoTraverseClimbCommand;
import frc.robot.commands.climb.InitiateClimbCommand;
import frc.robot.commands.drive.DriveClimbCommand;
import frc.robot.commands.shooter.CheckLeftCommand;
import frc.robot.commands.shooter.StrykeShotCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class TraverseClimbCommandGroup extends SequentialCommandGroup {
  public TraverseClimbCommandGroup(
      ClimbSubsystem climbSubsystem,
      DriveSubsystem driveSubsystem,
      ShooterSubsystem shooterSubsystem,
      MagazineSubsystem magazineSubsystem,
      Joystick driveJoystick,
      TurretSubsystem turretSubsystem) {
    addCommands(
        new ParallelCommandGroup(
            new InitiateClimbCommand(climbSubsystem, turretSubsystem),
            new DriveClimbCommand(driveSubsystem, driveJoystick, climbSubsystem)),
        new CheckLeftCommand(shooterSubsystem),
        new AutoTraverseClimbCommand(climbSubsystem),
        new StrykeShotCommand(turretSubsystem, shooterSubsystem, magazineSubsystem));
  }
}
