package frc.robot.commands.sequences.intaking;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.MagazineSubsystem.LowerMagazineState;

public class AutoIntakeCommand extends CommandBase {
  public final MagazineSubsystem magazineSubsystem;
  public final IntakeSubsystem intakeSubsystem;
  public boolean magazineReversed = false;
  public boolean isAuton;

  public AutoIntakeCommand(
      MagazineSubsystem magazineSubsystem, IntakeSubsystem intakeSubsystem, boolean isAuton) {
    addRequirements(magazineSubsystem, intakeSubsystem);
    this.magazineSubsystem = magazineSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.isAuton = isAuton;
  }

  @Override
  public void initialize() {
    magazineSubsystem.indexCargo();
    intakeSubsystem.openLoopRotate(IntakeConstants.kIntakeSpeed);
    magazineReversed = false;
  }

  @Override
  public void execute() {
    if (magazineSubsystem.getCurrLowerMagazineState() == LowerMagazineState.EJECT_CARGO
        && !magazineReversed) {
      intakeSubsystem.openLoopRotate(IntakeConstants.kIntakeEjectSpeed);
      magazineReversed = true;
    } else if (magazineReversed
        && (magazineSubsystem.getCurrLowerMagazineState() != LowerMagazineState.EJECT_CARGO)) {
      intakeSubsystem.openLoopRotate(IntakeConstants.kIntakeSpeed);
      magazineReversed = false;
    }
  }

  @Override
  public boolean isFinished() {
    return magazineSubsystem.isMagazineFull();
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted && !isAuton) {
      magazineSubsystem.magazineInterrupted();
      intakeSubsystem.openLoopRotate(0.0);
    } else if (!interrupted) {
      intakeSubsystem.openLoopRotate(IntakeConstants.kIntakeReverseSpeed);
    }
  }
}
