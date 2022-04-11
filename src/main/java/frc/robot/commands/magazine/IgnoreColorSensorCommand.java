package frc.robot.commands.magazine;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.MagazineSubsystem;

public class IgnoreColorSensorCommand extends InstantCommand {
  private final MagazineSubsystem magazineSubsystem;
  private final boolean ignore;

  public IgnoreColorSensorCommand(MagazineSubsystem magazineSubsystem, boolean ignore) {
    this.magazineSubsystem = magazineSubsystem;
    this.ignore = ignore;
  }

  @Override
  public void initialize() {
    magazineSubsystem.ignoreColorSensor(ignore);
  }
}
