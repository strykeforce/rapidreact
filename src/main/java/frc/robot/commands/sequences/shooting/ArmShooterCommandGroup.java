package frc.robot.commands.sequences.shooting;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.shooter.ArmShooterCommand;
import frc.robot.commands.turret.TurretAimCommandGroup;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class ArmShooterCommandGroup extends ParallelCommandGroup {
  public ArmShooterCommandGroup(
      VisionSubsystem visionSubsystem,
      TurretSubsystem turretSubsystem,
      ShooterSubsystem shooterSubsystem) {
    addCommands(
        new TurretAimCommandGroup(visionSubsystem, turretSubsystem),
        new ArmShooterCommand(shooterSubsystem));
  }
}
