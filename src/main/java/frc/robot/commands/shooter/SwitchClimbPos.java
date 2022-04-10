package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ShooterSubsystem;

public class SwitchClimbPos extends InstantCommand {
  public final ShooterSubsystem shooterSubsystem;

  public SwitchClimbPos(ShooterSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
  }

  @Override
  public void initialize() {
    shooterSubsystem.setOutside(!shooterSubsystem.isOutside);
  }
}
