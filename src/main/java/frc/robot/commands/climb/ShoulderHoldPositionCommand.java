package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ClimbSubsystem;

public class ShoulderHoldPositionCommand extends InstantCommand {
  private final ClimbSubsystem climbSubsystem;

  public ShoulderHoldPositionCommand(ClimbSubsystem climbSubsystem) {
    addRequirements(climbSubsystem);
    this.climbSubsystem = climbSubsystem;
  }

  @Override
  public void initialize() {
    // Do Nothing - just requires shoulder motor
  }
}
