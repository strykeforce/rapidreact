package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterOpenLoopCommand extends InstantCommand {
  private final ShooterSubsystem shooterSubsystem;
  private final double speed;

  public ShooterOpenLoopCommand(ShooterSubsystem shooterSubsystem, double speed) {
    addRequirements(shooterSubsystem);
    this.speed = speed;
    this.shooterSubsystem = shooterSubsystem;
  }

  @Override
  public void initialize() {
    shooterSubsystem.shooterOpenLoop(speed);
  }
}
