package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ClimbSubsystem;

public class OpenLoopFixedArmCommand extends InstantCommand {

  double speed;
  private final ClimbSubsystem climbSubsystem;

  public OpenLoopFixedArmCommand(ClimbSubsystem climbSubsystem, double speed) {
    addRequirements(climbSubsystem);
    this.climbSubsystem = climbSubsystem;
    this.speed = speed;
  }

  @Override
  public void initialize() {
    climbSubsystem.openLoopFixedArm(speed);
  }
}
