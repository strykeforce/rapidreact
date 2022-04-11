package frc.robot.commands.sequences.shooting;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.RetractOnFullCommand;
import frc.robot.commands.sequences.intaking.AutoIntakeNoExtendCommandGroup;
import frc.robot.subsystems.IntakeExtendSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class VisionShootCommandGroup extends SequentialCommandGroup {

  public VisionShootCommandGroup(
      ShooterSubsystem shooterSubsystem,
      TurretSubsystem turretSubsystem,
      MagazineSubsystem magazineSubsystem,
      VisionSubsystem visionSubsystem,
      boolean disableTrackingOnFinish,
      IntakeSubsystem intakeSubsystem,
      IntakeExtendSubsystem intakeExtendSubsystem,
      XboxController xbox) {
    addCommands(
        new ParallelDeadlineGroup(
            new VisionShootCommand(
                shooterSubsystem,
                turretSubsystem,
                magazineSubsystem,
                visionSubsystem,
                disableTrackingOnFinish,
                intakeSubsystem),
            new RetractOnFullCommand(magazineSubsystem, intakeExtendSubsystem)),
        new ScheduleCommand(
            new AutoIntakeNoExtendCommandGroup(magazineSubsystem, intakeSubsystem, xbox)),
        new ScheduleCommand(new RetractOnFullCommand(magazineSubsystem, intakeExtendSubsystem)));
  }
}
