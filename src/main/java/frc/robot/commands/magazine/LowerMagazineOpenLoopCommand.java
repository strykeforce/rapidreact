package frc.robot.commands.magazine;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.MagazineSubsystem;

public class LowerMagazineOpenLoopCommand extends InstantCommand {
  private final MagazineSubsystem magazineSubsystem;
  private final double speed;

  public LowerMagazineOpenLoopCommand(MagazineSubsystem magazineSubsystem, double speed) {
    addRequirements(magazineSubsystem);
    this.magazineSubsystem = magazineSubsystem;
    this.speed = speed;
  }

  @Override
  public void initialize() {
    magazineSubsystem.manualLowerMagazine(speed);
  }
}
