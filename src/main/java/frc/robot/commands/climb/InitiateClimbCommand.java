package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;

public class InitiateClimbCommand extends CommandBase {
  private final ClimbSubsystem climbSubsystem;

  public InitiateClimbCommand(ClimbSubsystem climbSubsystem) {
    addRequirements(climbSubsystem);
    this.climbSubsystem = climbSubsystem;
  }

  @Override
  public void initialize() {
    climbSubsystem.initiateClimb();
  }

  @Override
  public boolean isFinished() {
    return climbSubsystem.isClimbInitiated();
  }
}
