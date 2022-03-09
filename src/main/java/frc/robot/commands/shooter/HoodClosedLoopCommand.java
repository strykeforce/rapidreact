package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ShooterSubsystem;

public class HoodClosedLoopCommand extends InstantCommand {
  private final ShooterSubsystem shooterSubsystem;
  private final double setpointTicks;

  public HoodClosedLoopCommand(ShooterSubsystem shooterSubsystem, double setpointTicks) {
    addRequirements(shooterSubsystem);
    this.shooterSubsystem = shooterSubsystem;
    this.setpointTicks = setpointTicks;
  }

  @Override
  public void initialize() {
    shooterSubsystem.hoodClosedLoop(setpointTicks);
  }
}
