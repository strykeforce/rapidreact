package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class WaitMatchTimeCommand extends CommandBase {
  private final double waitUntil;

  public WaitMatchTimeCommand(double waitUntil) {
    this.waitUntil = waitUntil;
  }

  @Override
  public boolean isFinished() {
    return DriverStation.getMatchTime() <= waitUntil;
  }
}
