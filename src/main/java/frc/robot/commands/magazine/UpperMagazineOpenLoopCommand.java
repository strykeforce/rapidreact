package frc.robot.commands.magazine;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.MagazineSubsystem;

public class UpperMagazineOpenLoopCommand extends InstantCommand {
  private final MagazineSubsystem magazineSubsystem;
  private final double speed;

  public UpperMagazineOpenLoopCommand(MagazineSubsystem magazineSubsystem, double speed) {
    addRequirements(magazineSubsystem);
    this.magazineSubsystem = magazineSubsystem;
    this.speed = speed;
  }

  @Override
  public void initialize() {
    magazineSubsystem.upperOpenLoopRotate(speed);
  }
}
