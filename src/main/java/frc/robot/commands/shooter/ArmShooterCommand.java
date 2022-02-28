package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;

public class ArmShooterCommand extends CommandBase {
  public ShooterSubsystem shooterSubsystem;

  public ArmShooterCommand(ShooterSubsystem shooterSubsystem) {
    addRequirements(shooterSubsystem);
    this.shooterSubsystem = shooterSubsystem;
  }

  @Override
  public void initialize() {
    shooterSubsystem.arm();
  }

  @Override
  public boolean isFinished() {
    return shooterSubsystem.getCurrentState() == ShooterState.ARMED;
  }
}
