package frc.robot.commands.magazine;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.MagazineConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ManualEjectCargoReverseCommand extends InstantCommand {
  private final MagazineSubsystem magazineSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final ShooterSubsystem shooterSubsystem;

  public ManualEjectCargoReverseCommand(
      MagazineSubsystem magazineSubsystem,
      IntakeSubsystem intakeSubsystem,
      ShooterSubsystem shooterSubsystem) {
    addRequirements(magazineSubsystem, intakeSubsystem, shooterSubsystem);
    this.magazineSubsystem = magazineSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.shooterSubsystem = shooterSubsystem;
  }

  @Override
  public void initialize() {
    magazineSubsystem.manualEjectCargoReverse(
        MagazineConstants.kLowerMagazineEjectSpeed, MagazineConstants.kUpperMagazineEjectSpeed);
    intakeSubsystem.openLoopRotate(IntakeConstants.kIntakeEjectSpeed);
    shooterSubsystem.manualClosedLoop(
        ShooterConstants.kKickerManualEjectTicksP100ms,
        ShooterConstants.kShooterManualEjectTicksP100ms);
  }
}
