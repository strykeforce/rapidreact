package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class StopTurretCommand extends CommandBase {
  public final TurretSubsystem turretSubsystem;

  public StopTurretCommand(TurretSubsystem turretSubsystem) {
    addRequirements(turretSubsystem);
    this.turretSubsystem = turretSubsystem;
  }

  @Override
  public void initialize() {
    turretSubsystem.stopTrackingTarget();
  }
}
