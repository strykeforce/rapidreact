package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;

public class AutoHighClimbCommand extends CommandBase {
  private final ClimbSubsystem climbSubsystem;

  public AutoHighClimbCommand(ClimbSubsystem climbSubsystem) {
    addRequirements(climbSubsystem);
    this.climbSubsystem = climbSubsystem;
  }

  @Override
  public void initialize() {
    climbSubsystem.highClimb();
  }

  @Override
  public boolean isFinished() {
    return climbSubsystem.isClimbDone();
  }
}
