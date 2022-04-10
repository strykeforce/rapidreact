package frc.robot.commands.sequences.intaking;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeExtendSubsystem;

public class ExtendIntakeCommand extends CommandBase {
  private final IntakeExtendSubsystem intakeExtendSubsystem;
  private final boolean extend;

  public ExtendIntakeCommand(IntakeExtendSubsystem intakeExtendSubsystem, boolean extend) {
    addRequirements(intakeExtendSubsystem);
    this.intakeExtendSubsystem = intakeExtendSubsystem;
    this.extend = extend;
  }

  @Override
  public void initialize() {
    if (extend) intakeExtendSubsystem.extendClosedLoop();
    else intakeExtendSubsystem.retractClosedLoop();
  }

  @Override
  public boolean isFinished() {
    return intakeExtendSubsystem.isIntakeAtPos();
  }
}
