package frc.robot.commands.climb;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class InitiateClimbCommand extends CommandBase {
  private final ClimbSubsystem climbSubsystem;
  private final TurretSubsystem turretSubsystem;

  public InitiateClimbCommand(ClimbSubsystem climbSubsystem, TurretSubsystem turretSubsystem) {
    addRequirements(climbSubsystem, turretSubsystem);
    this.climbSubsystem = climbSubsystem;
    this.turretSubsystem = turretSubsystem;
  }

  @Override
  public void initialize() {
    climbSubsystem.initiateClimb();
    turretSubsystem.stopTrackingTarget();
    turretSubsystem.rotateTo(Rotation2d.fromDegrees(0.0));
  }

  @Override
  public boolean isFinished() {
    return climbSubsystem.isClimbInitiated();
  }
}
