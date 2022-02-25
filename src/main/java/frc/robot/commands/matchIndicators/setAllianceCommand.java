package frc.robot.commands.matchIndicators;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class setAllianceCommand extends InstantCommand {
  public final Alliance alliance;
  public final RobotContainer robotContainer;

  public setAllianceCommand(Alliance alliance, RobotContainer robotContainer) {

    this.alliance = alliance;
    this.robotContainer = robotContainer;
  }

  @Override
  public void initialize() {
    robotContainer.setAllianceColor(alliance);
  }
}
