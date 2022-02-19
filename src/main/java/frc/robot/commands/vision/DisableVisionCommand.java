package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.VisionSubsystem;

public class DisableVisionCommand extends InstantCommand {
  private final VisionSubsystem visionSubsystem;

  public DisableVisionCommand(VisionSubsystem visionSubsystem) {
    this.visionSubsystem = visionSubsystem;
    addRequirements(visionSubsystem);
  }

  @Override
  public void initialize() {
    visionSubsystem.disable();
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
