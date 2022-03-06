package frc.robot.commands.magazine;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.MagazineConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MagazineSubsystem;

public class ManualEjectCargoReverseCommand extends InstantCommand {
  private final MagazineSubsystem magazineSubsystem;
  private final IntakeSubsystem intakeSubsystem;

  public ManualEjectCargoReverseCommand(
      MagazineSubsystem magazineSubsystem, IntakeSubsystem intakeSubsystem) {
    addRequirements(magazineSubsystem);
    this.magazineSubsystem = magazineSubsystem;
    this.intakeSubsystem = intakeSubsystem;
  }

  @Override
  public void initialize() {
    magazineSubsystem.manualEjectCargoReverse(
        MagazineConstants.kMagazineEjectSpeed, MagazineConstants.kMagazineEjectSpeed);
    intakeSubsystem.openLoopRotate(IntakeConstants.kIntakeEjectSpeed);
  }
}
