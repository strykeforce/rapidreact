package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class PitIntakeOpenLoopCommand extends InstantCommand {
  private final IntakeSubsystem intakeSubsystem;

  public PitIntakeOpenLoopCommand(IntakeSubsystem intakeSubsystem) {
    addRequirements(intakeSubsystem);
    this.intakeSubsystem = intakeSubsystem;
  }

  @Override
  public void initialize() {
    intakeSubsystem.openLoopRotate(SmartDashboard.getNumber("Pit/Intake/Speed", 0));
  }
}
