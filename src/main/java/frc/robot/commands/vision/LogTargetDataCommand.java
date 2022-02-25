package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.VisionSubsystem;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class LogTargetDataCommand extends SequentialCommandGroup {

  private final Logger logger = LoggerFactory.getLogger(this.getClass());

  public LogTargetDataCommand(VisionSubsystem visionSubsystem) {
    addCommands(
        new EnableVisionCommand(visionSubsystem),
        new WaitCommand(0.5),
        new InstantCommand(
            () -> logger.info("{}", visionSubsystem.getTargetData()), visionSubsystem),
        new DisableVisionCommand(visionSubsystem));
  }
}
