package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class EnableVisionCommand extends InstantCommand {
  private final VisionSubsystem visionSubsystem;
  private final DriveSubsystem driveSubsystem;

  public EnableVisionCommand(VisionSubsystem visionSubsystem, DriveSubsystem driveSubsystem) {
    this.visionSubsystem = visionSubsystem;
    this.driveSubsystem = driveSubsystem;
    addRequirements(visionSubsystem);
  }

  @Override
  public void initialize() {
    visionSubsystem.enable();
    driveSubsystem.doOdomReset();
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
