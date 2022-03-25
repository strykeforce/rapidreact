package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.OdometryTestSubsystem;

public class OdometryTestSetPosition extends InstantCommand {
  public double pos;
  public final OdometryTestSubsystem odometryTestSubsystem;

  public OdometryTestSetPosition(OdometryTestSubsystem odometryTestSubsystem, double pos) {
    this.pos = pos;
    this.odometryTestSubsystem = odometryTestSubsystem;
  }

  @Override
  public void initialize() {
    odometryTestSubsystem.setPos(pos);
  }
}
