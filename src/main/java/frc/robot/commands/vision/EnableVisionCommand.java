package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.VisionSubsystem;

public class EnableVisionCommand extends InstantCommand{
    private final VisionSubsystem visionSubsystem;
    public EnableVisionCommand(VisionSubsystem visionSubsystem) {
        this.visionSubsystem = visionSubsystem;
        addRequirements(visionSubsystem);
    }
    @Override
    public void initialize() {
        visionSubsystem.enable();
    }
    
}
