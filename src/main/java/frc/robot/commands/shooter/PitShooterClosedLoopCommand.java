package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.SmartDashboardConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class PitShooterClosedLoopCommand extends InstantCommand {
  private final ShooterSubsystem shooterSubsystem;

  public PitShooterClosedLoopCommand(ShooterSubsystem shooterSubsystem) {
    addRequirements(shooterSubsystem);
    this.shooterSubsystem = shooterSubsystem;
  }

  @Override
  public void initialize() {
    shooterSubsystem.shooterClosedLoop(
        SmartDashboard.getNumber(SmartDashboardConstants.kPitKickerSetPointTicks, 0.0),
        SmartDashboard.getNumber(SmartDashboardConstants.kPitShooterSetPointTicks, 0.0));
  }
}
