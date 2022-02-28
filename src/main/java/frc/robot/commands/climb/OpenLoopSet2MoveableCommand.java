package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ClimbSubsystem;

public class OpenLoopSet2MoveableCommand extends InstantCommand {

  double speed;
  private final ClimbSubsystem climbSubsystem;

  public OpenLoopSet2MoveableCommand(ClimbSubsystem climbSubsystem, double speed) {
    addRequirements(climbSubsystem);
    this.climbSubsystem = climbSubsystem;
    this.speed = speed;
  }

  @Override
  public void initialize() {
    climbSubsystem.openLoopSet2Moveable(speed);
  }
}
