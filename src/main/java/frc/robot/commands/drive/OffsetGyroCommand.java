package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;

public class OffsetGyroCommand extends InstantCommand {
  public final DriveSubsystem driveSubsystem;
  public Rotation2d gyroOffset;

  public OffsetGyroCommand(DriveSubsystem driveSubsystem, Rotation2d gyroOffset) {
    addRequirements(driveSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.gyroOffset = gyroOffset;
  }

  @Override
  public void initialize() {
    driveSubsystem.setGyroOffset(gyroOffset);
  }
}
