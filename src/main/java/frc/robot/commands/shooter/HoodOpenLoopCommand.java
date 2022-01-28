package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ShooterSubsystem;

public class HoodOpenLoopCommand extends InstantCommand {
  private ShooterSubsystem shooterSubsystem;
  private double speed;

  public HoodOpenLoopCommand(ShooterSubsystem shooterSubsystem, double speed) {
    addRequirements(shooterSubsystem);
    this.speed = speed;
    this.shooterSubsystem = shooterSubsystem;
  }

  @Override
  public void initialize() {
    shooterSubsystem.hoodOpenLoop(speed);
  }
}
