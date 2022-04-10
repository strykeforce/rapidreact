package frc.robot.commands.sequences.shooting;

import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.sequences.intaking.AutoIntakeNoExtendCommand;
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
      IntakeSubsystem intakeSubsystem) {
    addCommands(
        new VisionTimedShootCommand(
            shooterSubsystem,
            turretSubsystem,
            magazineSubsystem,
            visionSubsystem,
            disableTrackingOnFinish,
            intakeSubsystem),
        new ScheduleCommand(new AutoIntakeNoExtendCommand(magazineSubsystem, intakeSubsystem)));
  }
}
