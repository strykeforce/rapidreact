package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;

public class ExtendSailCommand extends CommandBase {
  private ClimbSubsystem climbSubsystem;

  public ExtendSailCommand(ClimbSubsystem climbSubsystem) {
    this.climbSubsystem = climbSubsystem;
    addRequirements(climbSubsystem);
  }

  @Override
  public void initialize() {
    climbSubsystem.extendSail();
  }

  @Override
  public boolean isFinished() {
    return climbSubsystem.getIsSailExtended();
  }
}
