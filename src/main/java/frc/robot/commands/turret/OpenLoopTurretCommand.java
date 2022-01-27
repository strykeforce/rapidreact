package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.TurretSubsystem;

public class OpenLoopTurretCommand extends InstantCommand {
  public TurretSubsystem turretSubsystem;
  public double speed;

  public OpenLoopTurretCommand(TurretSubsystem turretSubsystem, double speed) {
    addRequirements(turretSubsystem);
    this.turretSubsystem = turretSubsystem;
    this.speed = speed;
  }

  @Override
  public void initialize() {
    turretSubsystem.openLoopRotate(speed);
  }
}
