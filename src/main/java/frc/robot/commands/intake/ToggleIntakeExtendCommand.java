package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeExtendSubsystem;

public class ToggleIntakeExtendCommand extends CommandBase {
  private final IntakeExtendSubsystem intakeExtendSubsystem;

  public ToggleIntakeExtendCommand(IntakeExtendSubsystem intakeExtendSubsystem) {
    addRequirements(intakeExtendSubsystem);
    this.intakeExtendSubsystem = intakeExtendSubsystem;
  }

  @Override
  public void initialize() {
    if (intakeExtendSubsystem.getIsIntakeExtended()) intakeExtendSubsystem.retractClosedLoop();
    else intakeExtendSubsystem.extendClosedLoop();
  }

  @Override
  public boolean isFinished() {
    return intakeExtendSubsystem.isIntakeAtPos();
  }
}
