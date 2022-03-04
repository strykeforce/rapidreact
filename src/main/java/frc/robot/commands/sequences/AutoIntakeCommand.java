package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.MagazineSubsystem.MagazineState;

public class AutoIntakeCommand extends CommandBase {
  public final MagazineSubsystem magazineSubsystem;
  public final IntakeSubsystem intakeSubsystem;
  public boolean magazineReversed = false;

  public AutoIntakeCommand(MagazineSubsystem magazineSubsystem, IntakeSubsystem intakeSubsystem) {
    addRequirements(magazineSubsystem, intakeSubsystem);
    this.magazineSubsystem = magazineSubsystem;
    this.intakeSubsystem = intakeSubsystem;
  }

  @Override
  public void initialize() {
    magazineSubsystem.indexCargo();
    intakeSubsystem.openLoopRotate(IntakeConstants.kIntakeSpeed);
    magazineReversed = false;
  }

  @Override
  public void execute() {
    if (magazineSubsystem.getCurrMagazineState() == MagazineState.EJECT_CARGO) {
      intakeSubsystem.openLoopRotate(IntakeConstants.kIntakeEjectSpeed);
      magazineReversed = true;
    } else if (magazineReversed == true) {
      intakeSubsystem.openLoopRotate(IntakeConstants.kIntakeSpeed);
    }
  }

  @Override
  public boolean isFinished() {
    return magazineSubsystem.isMagazineFull();
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      magazineSubsystem.magazineInterrupted();
    }

    intakeSubsystem.openLoopRotate(0.0);
  }
}
