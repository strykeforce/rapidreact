package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ClimbSubsystem;

public class OpenLoopPivotArmCommand extends InstantCommand {

  double speed;
  private final ClimbSubsystem climbSubsystem;

  public OpenLoopPivotArmCommand(ClimbSubsystem climbSubsystem, double speed) {
    addRequirements(climbSubsystem);
    this.climbSubsystem = climbSubsystem;
    this.speed = speed;
  }

  @Override
  public void initialize() {
    climbSubsystem.openLoopPivotArm(speed);
  }
}
