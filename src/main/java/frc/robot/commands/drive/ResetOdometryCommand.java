package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;

public class ResetOdometryCommand extends InstantCommand {
  private final DriveSubsystem driveSubsystem;
  private final Pose2d pose;

  public ResetOdometryCommand(DriveSubsystem driveSubsystem, Pose2d pose) {
    addRequirements(driveSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.pose = pose;
  }

  @Override
  public void initialize() {
    driveSubsystem.resetOdometry(pose);
  }
}
