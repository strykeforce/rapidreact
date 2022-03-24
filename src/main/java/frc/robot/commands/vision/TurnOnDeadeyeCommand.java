package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.VisionSubsystem;

public class TurnOnDeadeyeCommand extends InstantCommand {
  private final VisionSubsystem visionSubsystem;

  public TurnOnDeadeyeCommand(VisionSubsystem visionSubsystem) {
    this.visionSubsystem = visionSubsystem;
    addRequirements(visionSubsystem);
  }

  @Override
  public void initialize() {
    visionSubsystem.turnOnDeadeye();
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
