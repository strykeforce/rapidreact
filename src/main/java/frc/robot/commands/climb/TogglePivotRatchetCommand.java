package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ClimbSubsystem;

public class TogglePivotRatchetCommand extends InstantCommand {
  private final ClimbSubsystem climbSubsystem;

  public TogglePivotRatchetCommand(ClimbSubsystem climbSubsystem) {
    addRequirements(climbSubsystem);
    this.climbSubsystem = climbSubsystem;
  }

  @Override
  public void initialize() {
    climbSubsystem.togglePivotArmRatchet();
  }
}
