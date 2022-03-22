package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class TurretFeedFwdTuneCommand extends CommandBase {
  private final TurretSubsystem turretSubsystem;

  public TurretFeedFwdTuneCommand(TurretSubsystem turretSubsystem) {
    addRequirements(turretSubsystem);
    this.turretSubsystem = turretSubsystem;
  }

  @Override
  public void execute() {
    turretSubsystem.updateOpenLoopFeedFwd();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
