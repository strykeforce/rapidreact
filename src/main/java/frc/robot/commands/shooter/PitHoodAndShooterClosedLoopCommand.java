package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.DashboardConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class PitHoodAndShooterClosedLoopCommand extends InstantCommand {
  private ShooterSubsystem shooterSubsystem;

  public PitHoodAndShooterClosedLoopCommand(ShooterSubsystem shooterSubsystem) {
    addRequirements(shooterSubsystem);
    this.shooterSubsystem = shooterSubsystem;
  }

  @Override
  public void initialize() {
    shooterSubsystem.manualClosedLoop(
        SmartDashboard.getNumber(DashboardConstants.kPitKickerSetpointTicks, 0.0),
        SmartDashboard.getNumber(DashboardConstants.kPitShooterSetpointTicks, 0.0));

    shooterSubsystem.hoodClosedLoop(
        SmartDashboard.getNumber(DashboardConstants.kPitHoodSetpointTicks, 0.0));
  }
}
