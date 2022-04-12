package frc.robot.commands.sequences.intaking;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.magazine.RumbleOnCargoCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MagazineSubsystem;

public class AutoIntakeNoExtendCommandGroup extends ParallelCommandGroup {

  public AutoIntakeNoExtendCommandGroup(
      MagazineSubsystem magazineSubsystem, IntakeSubsystem intakeSubsystem, XboxController xbox) {
    addCommands(
        new AutoIntakeNoExtendCommand(magazineSubsystem, intakeSubsystem),
        new RumbleOnCargoCommand(xbox, magazineSubsystem));
  }
}
