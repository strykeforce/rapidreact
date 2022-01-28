package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ShooterSubsystem;

public class PitShooterOpenLoopCommand extends InstantCommand {
  private ShooterSubsystem shooterSubsystem;

  public PitShooterOpenLoopCommand(ShooterSubsystem shooterSubsystem) {
    addRequirements(shooterSubsystem);
    this.shooterSubsystem = shooterSubsystem;
  }

  @Override
  public void initialize() {
    shooterSubsystem.shooterOpenLoop(SmartDashboard.getNumber("Pit/Shooter/shooterSpeed", 0.0));
  }
}