package frc.robot.commands.magazine;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.MagazineConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoSlowEjectCargoCommand extends CommandBase {
  private final MagazineSubsystem magazineSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private Timer slowEjectTimer = new Timer();

  public AutoSlowEjectCargoCommand(
      MagazineSubsystem magazineSubsystem,
      IntakeSubsystem intakeSubsystem,
      ShooterSubsystem shooterSubsystem) {
    addRequirements(magazineSubsystem, intakeSubsystem, shooterSubsystem);
    this.magazineSubsystem = magazineSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.shooterSubsystem = shooterSubsystem;
  }

  @Override
  public void initialize() {
    slowEjectTimer.reset();
    slowEjectTimer.start();
  }

  @Override
  public void execute() {
    magazineSubsystem.manualEjectCargoReverse(
        MagazineConstants.kLowerMagazineEjectSpeed, MagazineConstants.kUpperMagazineEjectSpeed);
    intakeSubsystem.openLoopRotate(IntakeConstants.kIntakeSlowEjectSpeed);
    shooterSubsystem.manualClosedLoop(
        ShooterConstants.kKickerManualEjectTicksP100ms,
        ShooterConstants.kShooterManualEjectTicksP100ms);
  }

  @Override
  public boolean isFinished() {
    return slowEjectTimer.get() >= Constants.AutoConstants.kSlowEjectTimerDelay;
  }

  @Override
  public void end(boolean interrupted) {
    slowEjectTimer.stop();
    magazineSubsystem.manualEjectCargoReverse(0.0, 0.0);
    intakeSubsystem.openLoopRotate(0.0);
    shooterSubsystem.manualClosedLoop(0.0, 0.0);
  }
}
