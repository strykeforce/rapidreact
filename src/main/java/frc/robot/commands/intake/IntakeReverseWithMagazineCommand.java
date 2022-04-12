package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.MagazineSubsystem.LowerMagazineState;

public class IntakeReverseWithMagazineCommand extends CommandBase {
  private final IntakeSubsystem intakeSubsystem;
  private final MagazineSubsystem magazineSubsystem;
  private boolean isReversed = false;

  public IntakeReverseWithMagazineCommand(
      MagazineSubsystem magazineSubsystem, IntakeSubsystem intakeSubsystem) {
    addRequirements(intakeSubsystem);
    this.intakeSubsystem = intakeSubsystem;
    this.magazineSubsystem = magazineSubsystem;
  }

  @Override
  public void initialize() {
    if (magazineSubsystem.getCurrLowerMagazineState() == LowerMagazineState.EJECT_CARGO) {
      intakeSubsystem.openLoopRotate(IntakeConstants.kIntakeEjectSpeed);
      isReversed = true;
    } else {
      intakeSubsystem.openLoopRotate(IntakeConstants.kIntakeSpeed);
      isReversed = false;
    }
  }

  @Override
  public void execute() {
    if (magazineSubsystem.getCurrLowerMagazineState() == LowerMagazineState.EJECT_CARGO
        && !isReversed) {
      intakeSubsystem.openLoopRotate(IntakeConstants.kIntakeEjectSpeed);
      isReversed = true;
    } else if (magazineSubsystem.getCurrLowerMagazineState() != LowerMagazineState.EJECT_CARGO
        && isReversed) {
      intakeSubsystem.openLoopRotate(IntakeConstants.kIntakeSpeed);
      isReversed = false;
    }
  }

  @Override
  public boolean isFinished() {
    return magazineSubsystem.isMagazineFull()
        && magazineSubsystem.getCurrLowerMagazineState() == LowerMagazineState.WAIT_UPPER;
  }

  @Override
  public void end(boolean interrupted) {
    if (!interrupted) intakeSubsystem.openLoopRotate(IntakeConstants.kIntakeReverseSpeed);
  }
}
