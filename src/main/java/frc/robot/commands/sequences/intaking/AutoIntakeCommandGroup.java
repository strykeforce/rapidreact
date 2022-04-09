package frc.robot.commands.sequences.intaking;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.sequences.shooting.ArmShooterCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AutoIntakeCommandGroup extends SequentialCommandGroup {
  public AutoIntakeCommandGroup(
      MagazineSubsystem magazineSubsystem,
      IntakeSubsystem intakeSubsystem,
      VisionSubsystem visionSubsystem,
      TurretSubsystem turretSubsystem,
      ShooterSubsystem shooterSubsystem,
      boolean isAuton,
      boolean extendArm) {
    addCommands(
        new AutoIntakeCommand(magazineSubsystem, intakeSubsystem, isAuton, extendArm),
        new ConditionalCommand(
            new ArmShooterCommandGroup(visionSubsystem, turretSubsystem, shooterSubsystem),
            new InstantCommand(),
            () -> magazineSubsystem.isMagazineFull()));
  }
}
