package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class ZeroTurretCommand extends CommandBase {
  public TurretSubsystem turretSubsystem;

  public ZeroTurretCommand(TurretSubsystem turretSubsystem) {
    addRequirements(turretSubsystem);
    this.turretSubsystem = turretSubsystem;
  }

  @Override
  public void initialize() {
    turretSubsystem.setTurret(0);
  }

  @Override
  public boolean isFinished() {
    return turretSubsystem.turretAtTarget();
  }
}
