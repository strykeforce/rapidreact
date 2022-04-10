package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class RotateToTickCommand extends CommandBase {
  private final TurretSubsystem turretSubsystem;
  private final double ticks;

  public RotateToTickCommand(TurretSubsystem turretSubsystem, double ticks) {
    this.turretSubsystem = turretSubsystem;
    this.ticks = ticks;
    addRequirements(turretSubsystem);
  }

  @Override
  public void initialize() {
    turretSubsystem.rotateTo(ticks);
  }

  @Override
  public boolean isFinished() {
    return turretSubsystem.isTurretAtTarget();
  }
}
