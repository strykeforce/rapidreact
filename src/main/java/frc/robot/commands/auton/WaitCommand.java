package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class WaitCommand extends CommandBase {
  private Timer timer = new Timer();
  private double delay;

  public WaitCommand(double delay) {
    this.delay = delay;
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(delay);
  }
}
