package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeOpenLoopCommand extends InstantCommand {
  private IntakeSubsystem intakeSubsystem;
  private double speed;

  public IntakeOpenLoopCommand(IntakeSubsystem intakeSubsystem, double speed) {
    addRequirements(intakeSubsystem);
    this.intakeSubsystem = intakeSubsystem;
    this.speed = speed;
  }

  @Override
  public void initialize() {
    intakeSubsystem.openLoopRotate(speed);
  }
}
