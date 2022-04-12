package frc.robot.commands.sequences.intaking;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.MagazineSubsystem.LowerMagazineState;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class AutoIntakeNoExtendCommand extends CommandBase {
  public final MagazineSubsystem magazineSubsystem;
  public final IntakeSubsystem intakeSubsystem;
  public boolean magazineReversed = false;
  private final Logger logger = LoggerFactory.getLogger(AutoIntakeNoExtendCommandGroup.class);

  public AutoIntakeNoExtendCommand(
      MagazineSubsystem magazineSubsystem, IntakeSubsystem intakeSubsystem) {
    addRequirements(magazineSubsystem, intakeSubsystem);
    this.magazineSubsystem = magazineSubsystem;
    this.intakeSubsystem = intakeSubsystem;
  }

  @Override
  public void initialize() {
    logger.info("Init");
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
    return magazineSubsystem.isMagazineFull()
        && magazineSubsystem.getCurrLowerMagazineState() == LowerMagazineState.WAIT_UPPER;
  }

  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      intakeSubsystem.openLoopRotate(IntakeConstants.kIntakeReverseSpeed);
    }
  }
}
