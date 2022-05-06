package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;

public class RetractSailCommand extends CommandBase {
  private ClimbSubsystem climbSubsystem;

  public RetractSailCommand(ClimbSubsystem climbSubsystem) {
    this.climbSubsystem = climbSubsystem;
    addRequirements(climbSubsystem);
  }

  @Override
  public void initialize() {
    climbSubsystem.retractSail();
  }

  @Override
  public boolean isFinished() {
    return !climbSubsystem.getIsSailExtended();
  }
}
