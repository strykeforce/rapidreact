package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ClimbSubsystem;

public class ToggleFixedRatchetCommand extends InstantCommand {

  private final ClimbSubsystem climbSubsystem;

  public ToggleFixedRatchetCommand(ClimbSubsystem climbSubsystem) {
    this.climbSubsystem = climbSubsystem;
    addRequirements(climbSubsystem);
  }

  @Override
  public void initialize() {
    climbSubsystem.toggleFixedArmRatchet();
  }
}
