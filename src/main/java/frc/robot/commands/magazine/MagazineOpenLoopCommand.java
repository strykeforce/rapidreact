package frc.robot.commands.magazine;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.MagazineSubsystem;

public class MagazineOpenLoopCommand extends InstantCommand {
  private MagazineSubsystem magazineSubsystem;
  private double speed;

  public MagazineOpenLoopCommand(MagazineSubsystem magazineSubsystem, double speed) {
    addRequirements(magazineSubsystem);
    this.magazineSubsystem = magazineSubsystem;
    this.speed = speed;
  }

  @Override
  public void initialize() {
    magazineSubsystem.openLoopRotate(speed);
  }
}
