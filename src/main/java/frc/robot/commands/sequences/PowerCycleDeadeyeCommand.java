package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.PowerDistHub;

public class PowerCycleDeadeyeCommand extends InstantCommand {
  private final PowerDistHub powerDistHub;

  public PowerCycleDeadeyeCommand(PowerDistHub powerDistHub) {
    addRequirements(powerDistHub);
    this.powerDistHub = powerDistHub;
  }

  @Override
  public void initialize() {
    powerDistHub.powerCycleDeadeye();
  }
}
