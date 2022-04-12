package frc.robot.commands.sequences.shooting;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.IntakeReverseWithMagazineCommand;
import frc.robot.commands.sequences.intaking.AutoIntakeNoExtendCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class VisionTimedShootCommandGroup extends SequentialCommandGroup {

  public VisionTimedShootCommandGroup(
      ShooterSubsystem shooterSubsystem,
      TurretSubsystem turretSubsystem,
      MagazineSubsystem magazineSubsystem,
      VisionSubsystem visionSubsystem,
      boolean disableTrackingOnFinish,
      IntakeSubsystem intakeSubsystem,
      XboxController xbox) {
    addCommands(
        new ParallelDeadlineGroup(
            new VisionTimedShootCommand(
                shooterSubsystem,
                turretSubsystem,
                magazineSubsystem,
                visionSubsystem,
                disableTrackingOnFinish),
            new IntakeReverseWithMagazineCommand(magazineSubsystem, intakeSubsystem)),
        new ScheduleCommand(
            new AutoIntakeNoExtendCommandGroup(magazineSubsystem, intakeSubsystem, xbox)));
  }
}
