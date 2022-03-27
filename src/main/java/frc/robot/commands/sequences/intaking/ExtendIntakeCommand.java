package frc.robot.commands.sequences.intaking;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class ExtendIntakeCommand extends CommandBase {
  private final IntakeSubsystem intakeSubsystem;
  private final boolean extend;

  public ExtendIntakeCommand(IntakeSubsystem intakeSubsystem, boolean extend) {
    addRequirements(intakeSubsystem);
    this.intakeSubsystem = intakeSubsystem;
    this.extend = extend;
  }

  @Override
  public void initialize() {
    if (extend) intakeSubsystem.extendClosedLoop();
    else intakeSubsystem.retractClosedLoop();
  }

  @Override
  public boolean isFinished() {
    return intakeSubsystem.isIntakeAtPos();
  }
}
