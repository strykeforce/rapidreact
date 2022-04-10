package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ShooterSubsystem;

public class CheckLeftCommand extends InstantCommand {
  public final ShooterSubsystem shooterSubsystem;

  public CheckLeftCommand(ShooterSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
  }

  @Override
  public void initialize() {
    shooterSubsystem.checkOutside();
  }
}
