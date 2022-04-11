package frc.robot.commands.magazine;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DashboardConstants;
import frc.robot.subsystems.MagazineSubsystem;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class RumbleOnCargoCommand extends CommandBase {
  private final Timer rumbleTimer = new Timer();
  private final XboxController xbox;
  private final MagazineSubsystem magazineSubsystem;
  private boolean startedRumble = false;
  private final Logger logger = LoggerFactory.getLogger(RumbleOnCargoCommand.class);

  public RumbleOnCargoCommand(XboxController xbox, MagazineSubsystem magazineSubsystem) {
    this.xbox = xbox;
    this.magazineSubsystem = magazineSubsystem;
  }

  @Override
  public void initialize() {
    startedRumble = false;
  }

  @Override
  public void execute() {
    if (!magazineSubsystem.isMagazineEmpty()
        && !startedRumble
        && magazineSubsystem.isFirstCargoAlliance()) {
      rumbleTimer.reset();
      rumbleTimer.start();
      startedRumble = true;
      xbox.setRumble(RumbleType.kLeftRumble, 1.0);
      xbox.setRumble(RumbleType.kRightRumble, 1.0);
      logger.info("Start rumble");
    }
  }

  @Override
  public boolean isFinished() {
    return startedRumble && rumbleTimer.hasElapsed(DashboardConstants.kRumbleTime);
  }

  @Override
  public void end(boolean interrupted) {
    xbox.setRumble(RumbleType.kLeftRumble, 0.0);
    xbox.setRumble(RumbleType.kRightRumble, 0.0);
    logger.info("Stop rumble");
  }
}
