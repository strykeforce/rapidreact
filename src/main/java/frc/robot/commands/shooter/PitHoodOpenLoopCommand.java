package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.SmartDashboardConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class PitHoodOpenLoopCommand extends InstantCommand {
  private final ShooterSubsystem shooterSubsystem;

  public PitHoodOpenLoopCommand(ShooterSubsystem shooterSubsystem) {
    addRequirements(shooterSubsystem);
    this.shooterSubsystem = shooterSubsystem;
  }

  @Override
  public void initialize() {
    shooterSubsystem.hoodOpenLoop(
        SmartDashboard.getNumber(SmartDashboardConstants.kPitHoodOpenLoop, 0.0));
  }
}