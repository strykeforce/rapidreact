package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.vision.DisableVisionCommand;
import frc.robot.commands.vision.EnableVisionCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class TurretAimCommandGroup extends SequentialCommandGroup {
  public TurretAimCommandGroup(
      VisionSubsystem visionSubsystem,
      TurretSubsystem turretSubsystem,
      DriveSubsystem driveSubsystem) {
    addRequirements(visionSubsystem, turretSubsystem);
    addCommands(
        new EnableVisionCommand(visionSubsystem, driveSubsystem),
        new WaitCommand(0.5),
        new TurretAimCommand(turretSubsystem),
        new DisableVisionCommand(visionSubsystem));
  }
}
