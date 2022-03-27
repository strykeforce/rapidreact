package frc.robot.commands.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.vision.DisableVisionCommand;
import frc.robot.commands.vision.EnableVisionCommand;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class DeadeyeLatencyTestCommandGroup extends SequentialCommandGroup {

  public DeadeyeLatencyTestCommandGroup(
      VisionSubsystem visionSubsystem, TurretSubsystem turretSubsystem) {
    addCommands(
        new EnableVisionCommand(visionSubsystem),
        new WaitCommand(10.0),
        new RotateToCommand(turretSubsystem, Rotation2d.fromDegrees(90.0)),
        new RotateToCommand(turretSubsystem, Rotation2d.fromDegrees(-90.0)),
        // new RotateToCommand(turretSubsystem, Rotation2d.fromDegrees(90.0)),
        // new RotateToCommand(turretSubsystem, Rotation2d.fromDegrees(-90.0)),
        new DisableVisionCommand(visionSubsystem));
  }
}
