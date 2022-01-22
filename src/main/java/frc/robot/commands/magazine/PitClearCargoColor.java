package frc.robot.commands.magazine;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.MagazineSubsystem.CargoColor;

public class PitClearCargoColor extends InstantCommand {
  private MagazineSubsystem magazineSubsystem;

  public PitClearCargoColor(MagazineSubsystem magazineSubsystem) {
    addRequirements(magazineSubsystem);
    this.magazineSubsystem = magazineSubsystem;
  }

  @Override
  public void initialize() {
    magazineSubsystem.clearCargoColors();
    CargoColor[] storedCargo = magazineSubsystem.getAllCargoColors();
    SmartDashboard.putString("Pit/Magazine/First Cargo Color", storedCargo[0].toString());
    SmartDashboard.putString("Pit/Magazine/Second Cargo Color", storedCargo[1].toString());
  }
}
