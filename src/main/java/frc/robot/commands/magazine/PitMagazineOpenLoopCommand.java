package frc.robot.commands.magazine;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.MagazineSubsystem;

public class PitMagazineOpenLoopCommand extends InstantCommand {
  private MagazineSubsystem magazineSubsystem;

  public PitMagazineOpenLoopCommand(MagazineSubsystem magazineSubsystem) {
    addRequirements(magazineSubsystem);
    this.magazineSubsystem = magazineSubsystem;
  }

  @Override
  public void initialize() {
    magazineSubsystem.openLoopRotate(SmartDashboard.getNumber("Pit/Magazine/Speed", 0));
  }
}
