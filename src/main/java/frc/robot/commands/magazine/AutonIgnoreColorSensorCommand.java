package frc.robot.commands.magazine;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.MagazineSubsystem;

public class AutonIgnoreColorSensorCommand extends InstantCommand {
  private final MagazineSubsystem magazineSubsystem;
  private final boolean autonIgnore;

  public AutonIgnoreColorSensorCommand(MagazineSubsystem magazineSubsystem, boolean autonIgnore) {
    this.magazineSubsystem = magazineSubsystem;
    this.autonIgnore = autonIgnore;
  }

  @Override
  public void initialize() {
    magazineSubsystem.setAutonIgnoreColorSensor(autonIgnore);
  }
}
