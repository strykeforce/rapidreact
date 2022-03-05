package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ClimbSubsystem;

public class ToggleStaticRatchetPosCommand extends InstantCommand {

  private final ClimbSubsystem climbSubsystem;

  public ToggleStaticRatchetPosCommand(ClimbSubsystem climbSubsystem) {
    this.climbSubsystem = climbSubsystem;
    addRequirements(climbSubsystem);
  }

  @Override
  public void initialize() {
    climbSubsystem.toggleFixedArmRatchet();
  }
}
