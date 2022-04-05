package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class SeekCenterOdometryCommand extends CommandBase {
  private final TurretSubsystem turretSubsystem;

  public SeekCenterOdometryCommand(TurretSubsystem turretSubsystem) {
    addRequirements(turretSubsystem);
    this.turretSubsystem = turretSubsystem;
  }

  @Override
  public void initialize() {
    turretSubsystem.seekCenter();
  }

  @Override
  public boolean isFinished() {
    return turretSubsystem.isTurretAtTarget();
  }
}
