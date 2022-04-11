package frc.robot.commands.sequences.intaking;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.magazine.RumbleOnCargoCommand;
import frc.robot.subsystems.IntakeExtendSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MagazineSubsystem;

public class AutoIntakeCommandGroup extends ParallelCommandGroup {
  public AutoIntakeCommandGroup(
      MagazineSubsystem magazineSubsystem,
      IntakeSubsystem intakeSubsystem,
      IntakeExtendSubsystem intakeExtendSubsystem,
      XboxController xbox,
      boolean isAuton,
      boolean extendArm) {
    addCommands(
        new AutoIntakeCommand(
            magazineSubsystem, intakeSubsystem, intakeExtendSubsystem, isAuton, extendArm),
        new RumbleOnCargoCommand(xbox, magazineSubsystem));
  }
}
