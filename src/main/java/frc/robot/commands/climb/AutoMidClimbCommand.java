package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;

public class AutoMidClimbCommand extends CommandBase {
  private final ClimbSubsystem climbSubsystem;

  public AutoMidClimbCommand(ClimbSubsystem climbSubsystem) {
    addRequirements(climbSubsystem);
    this.climbSubsystem = climbSubsystem;
  }

  @Override
  public void initialize() {
    climbSubsystem.midClimb();
  }

  @Override
  public boolean isFinished() {
    return climbSubsystem.isClimbDone();
  }
}
