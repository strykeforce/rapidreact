package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.ClimbSubsystem;

public class RotateShoulderUpCommand extends CommandBase {
  private final ClimbSubsystem climbSubsystem;

  public RotateShoulderUpCommand(ClimbSubsystem climbSubsystem) {
    addRequirements(climbSubsystem);
    this.climbSubsystem = climbSubsystem;
  }

  @Override
  public void initialize() {
    climbSubsystem.offsetShoulder(ClimbConstants.kShoulderOffsetTicks);
  }

  @Override
  public void execute() {
    climbSubsystem.offsetShoulder(ClimbConstants.kShoulderOffsetTicks);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
