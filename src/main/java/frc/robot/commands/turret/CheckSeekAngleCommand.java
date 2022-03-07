package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.TurretSubsystem;

public class CheckSeekAngleCommand extends InstantCommand {

  public TurretSubsystem turretSubsystem;

  public CheckSeekAngleCommand(TurretSubsystem turretSubsystem) {
    this.turretSubsystem = turretSubsystem;
  }

  @Override
  public void initialize() {
    turretSubsystem.setSeekAngle(true);
  }
}
