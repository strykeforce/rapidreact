package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MagazineSubsystem;

public class AutoIntakeCommand extends CommandBase {
  public final MagazineSubsystem magazineSubsystem;
  public final IntakeSubsystem intakeSubsystem;

  public AutoIntakeCommand(MagazineSubsystem magazineSubsystem, IntakeSubsystem intakeSubsystem) {
    addRequirements(magazineSubsystem, intakeSubsystem);
    this.magazineSubsystem = magazineSubsystem;
    this.intakeSubsystem = intakeSubsystem;
  }

  @Override
  public void initialize() {
    magazineSubsystem.indexCargo();
    intakeSubsystem.openLoopRotate(IntakeConstants.kIntakeSpeed);
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
