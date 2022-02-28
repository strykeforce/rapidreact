package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.ClimbSubsystem;

public class RotateShoulderUpCommand extends InstantCommand {
  private final ClimbSubsystem climbSubsystem;

  public RotateShoulderUpCommand(ClimbSubsystem climbSubsystem) {
    this.climbSubsystem = climbSubsystem;
  }

  @Override
  public void initialize() {
    climbSubsystem.offsetShoulder(ClimbConstants.kShoulderOffsetTicks);
  }
}
