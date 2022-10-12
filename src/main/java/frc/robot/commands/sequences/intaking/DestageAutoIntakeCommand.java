package frc.robot.commands.sequences.intaking;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeExtendSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.MagazineSubsystem.LowerMagazineState;

public class DestageAutoIntakeCommand extends CommandBase {
  public final MagazineSubsystem magazineSubsystem;
  public final IntakeSubsystem intakeSubsystem;
  private final IntakeExtendSubsystem intakeExtendSubsystem;
  public boolean magazineReversed = false;
  public boolean isAuton;
  public boolean intakeExtend;

  public DestageAutoIntakeCommand(
      MagazineSubsystem magazineSubsystem,
      IntakeSubsystem intakeSubsystem,
      IntakeExtendSubsystem intakeExtendSubsystem,
      boolean isAuton,
      boolean intakeExtend) {
    addRequirements(magazineSubsystem, intakeSubsystem);
    this.magazineSubsystem = magazineSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.intakeExtendSubsystem = intakeExtendSubsystem;
    this.isAuton = isAuton;
    this.intakeExtend = intakeExtend;
  }

  @Override
  public void initialize() {
    magazineSubsystem.indexCargo();
    intakeSubsystem.openLoopRotate(
        isAuton ? IntakeConstants.kIntakeSpeedAuto : IntakeConstants.kIntakeSpeed);
    magazineReversed = false;
    if (intakeExtend) intakeExtendSubsystem.extendClosedLoop();
    else intakeExtendSubsystem.retractClosedLoop();
  }

  @Override
  public void execute() {
    if (magazineSubsystem.getCurrLowerMagazineState() == LowerMagazineState.EJECT_CARGO
        && !magazineReversed) {
      intakeSubsystem.openLoopRotate(IntakeConstants.kIntakeEjectSpeed);
      magazineReversed = true;
    } else if (magazineReversed
        && (magazineSubsystem.getCurrLowerMagazineState() != LowerMagazineState.EJECT_CARGO)) {
      intakeSubsystem.openLoopRotate(
          isAuton ? IntakeConstants.kIntakeSpeedAuto : IntakeConstants.kIntakeSpeed);
      magazineReversed = false;
    }
  }

  @Override
  public boolean isFinished() {
    return !magazineSubsystem.isMagazineEmpty();
  }

  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      intakeSubsystem.openLoopRotate(IntakeConstants.kIntakeReverseSpeed);
    }
    if (!isAuton && !interrupted) intakeExtendSubsystem.retractClosedLoop();
  }
}
