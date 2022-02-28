package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class SetAllianceCommand extends InstantCommand {
  public final Alliance alliance;
  public final RobotContainer robotContainer;

  public SetAllianceCommand(Alliance alliance, RobotContainer robotContainer) {

    this.alliance = alliance;
    this.robotContainer = robotContainer;
  }

  @Override
  public void initialize() {
    robotContainer.setAllianceColor(alliance);
  }
}
