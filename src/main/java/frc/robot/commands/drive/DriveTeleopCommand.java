package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class DriveTeleopCommand extends CommandBase {
  private final Joystick joystick;
  private final DriveSubsystem driveSubsystem;
  private final SlewRateLimiter fwdLimiter = new SlewRateLimiter(DriveConstants.kRateLimitFwdStr);
  private final SlewRateLimiter strLimiter = new SlewRateLimiter(DriveConstants.kRateLimitFwdStr);
  private final SlewRateLimiter yawLimiter = new SlewRateLimiter(DriveConstants.kRateLimitYaw);

  public DriveTeleopCommand(Joystick driver, DriveSubsystem driveSubsystem) {
    addRequirements(driveSubsystem);
    joystick = driver;
    this.driveSubsystem = driveSubsystem;
  }

  @Override
  public void execute() {
    driveSubsystem.drive(
        -DriveConstants.kMaxSpeedMetersPerSecond
            * fwdLimiter.calculate(
                MathUtil.applyDeadband(
                    joystick.getRawAxis(RobotContainer.Axis.LEFT_X.id),
                    DriveConstants.kDeadbandAllStick)),
        -DriveConstants.kMaxSpeedMetersPerSecond
            * strLimiter.calculate(
                MathUtil.applyDeadband(
                    joystick.getRawAxis(RobotContainer.Axis.LEFT_Y.id),
                    DriveConstants.kDeadbandAllStick)),
        -DriveConstants.kMaxOmega
            * yawLimiter.calculate(
                MathUtil.applyDeadband(
                    joystick.getRawAxis(RobotContainer.Axis.RIGHT_Y.id),
                    DriveConstants.kDeadbandAllStick)));
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0, 0, 0);
  }

  private double deadband(double stickValue) {
    if (Math.abs(stickValue) <= Constants.DriveConstants.kDeadbandAllStick) {
      return 0;
    }
    return stickValue;
  }
}
