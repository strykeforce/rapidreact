package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.vision.DisableVisionCommand;
import frc.robot.commands.vision.EnableVisionCommand;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class TurretAimCommandGroup extends SequentialCommandGroup {
  public TurretAimCommandGroup(VisionSubsystem visionSubsystem, TurretSubsystem turretSubsystem) {
    addRequirements(visionSubsystem, turretSubsystem);
    addCommands(
        new EnableVisionCommand(visionSubsystem),
        new WaitCommand(1.0),
        new TurretAimCommand(visionSubsystem, turretSubsystem),
        new DisableVisionCommand(visionSubsystem));
  }
}
