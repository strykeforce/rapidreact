package frc.robot.commands.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class RotateToCommand extends CommandBase {

  private final TurretSubsystem turretSubsystem;
  private final Rotation2d position;

  public RotateToCommand(TurretSubsystem turretSubsystem, Rotation2d position) {
    this.turretSubsystem = turretSubsystem;
    this.position = position;
    addRequirements(turretSubsystem);
  }

  @Override
  public void initialize() {
    turretSubsystem.rotateTo(position);
  }

  @Override
  public boolean isFinished() {
    return turretSubsystem.isRotationFinished();
  }
}
