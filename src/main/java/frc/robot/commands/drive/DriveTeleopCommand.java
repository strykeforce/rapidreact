package frc.robot.commands.drive;

import com.fasterxml.jackson.databind.jsonFormatVisitors.JsonObjectFormatVisitor;

import org.opencv.core.Mat;
import org.strykeforce.thirdcoast.util.ExpoScale;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.DashboardConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class DriveTeleopCommand extends CommandBase {
  private final Joystick joystick;
  private final DriveSubsystem driveSubsystem;
  private final ExpoScale expoScaleMovement = new ExpoScale(DashboardConstants.kLeftStickDeadBand, DriveConstants.kExpoScaleMoveFactor);
  private final ExpoScale expoScaleYaw = new ExpoScale(DashboardConstants.kRightStickDeadBand, DriveConstants.kExpoScaleYawFactor);
  private double rawForward;
  private double rawStrafe;
  private double rawMagnitude;
  private double rawAngle;
  private double adjustedMag;
  private double adjustedForward;
  private double adjustedStrafe;
  

  public DriveTeleopCommand(Joystick driver, DriveSubsystem driveSubsystem) {
    addRequirements(driveSubsystem);
    joystick = driver;
    this.driveSubsystem = driveSubsystem;
  }

  @Override
  public void execute() {

    rawForward = joystick.getRawAxis(RobotContainer.Axis.LEFT_X.id);
    rawStrafe = joystick.getRawAxis(RobotContainer.Axis.LEFT_Y.id);
    rawMagnitude = Math.sqrt(Math.pow(rawForward, 2)+Math.pow(rawStrafe,2));
    rawAngle = Math.atan(rawForward/rawStrafe);
    adjustedMag = expoScaleMovement.apply(rawMagnitude);
    adjustedForward = Math.sin(rawAngle) * adjustedMag;
    adjustedStrafe = Math.cos(rawAngle) * adjustedMag;
    
    




    driveSubsystem.drive(
        DriveConstants.kMaxSpeedMetersPerSecond
            * -MathUtil.applyDeadband(
                joystick.getRawAxis(RobotContainer.Axis.LEFT_X.id),
                DriveConstants.kDeadbandAllStick),
        DriveConstants.kMaxSpeedMetersPerSecond
            * -MathUtil.applyDeadband(
                joystick.getRawAxis(RobotContainer.Axis.LEFT_Y.id),
                DriveConstants.kDeadbandAllStick),
        DriveConstants.kMaxOmega
            * -MathUtil.applyDeadband(
                joystick.getRawAxis(RobotContainer.Axis.RIGHT_Y.id),
                DriveConstants.kDeadbandAllStick));
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
