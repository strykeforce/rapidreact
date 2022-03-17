package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.DashboardConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import org.strykeforce.thirdcoast.util.ExpoScale;

public class DriveTeleopCommand extends CommandBase {
  private final Joystick joystick;
  private final DriveSubsystem driveSubsystem;
  private final ExpoScale expoScaleYaw =
      new ExpoScale(DashboardConstants.kRightStickDeadBand, DriveConstants.kExpoScaleYawFactor);
  private double[] adjustedValues = new double[3];
  private final double vectorOffset = Math.sqrt(2)/(DriveConstants.kExpoScaleMoveFactor * Math.pow((Math.sqrt(2)-DriveConstants.kDeadbandAllStick), 3) + (1-DriveConstants.kExpoScaleMoveFactor) * (Math.sqrt(2) - DriveConstants.kDeadbandAllStick));

  public DriveTeleopCommand(Joystick driver, DriveSubsystem driveSubsystem) {
    addRequirements(driveSubsystem);
    joystick = driver;
    this.driveSubsystem = driveSubsystem;
  }

  private double applyVectorDeadBand(double input) {
    double y;

    if (Math.abs(input) < DriveConstants.kDeadbandAllStick) {
      return 0;
    }

    y = input > 0 ? input - DriveConstants.kDeadbandAllStick : input + DriveConstants.kDeadbandAllStick;
    return (DriveConstants.kExpoScaleMoveFactor * Math.pow(y, 3) + (1 - DriveConstants.kExpoScaleMoveFactor) * y) * vectorOffset;
  }

  private double[] calcAdjustedValues(double rawForward, double rawStrafe, double rawYaw) {
    double[] tempAdjustedValues = new double[3];
    double rawAngle = Math.atan2(rawForward , rawStrafe);
    double orgMag = (Math.sqrt(Math.pow(rawForward, 2) + Math.pow(rawStrafe, 2)));
    double adjustedMag = applyVectorDeadBand(orgMag);
    tempAdjustedValues[0] = Math.sin(rawAngle) * adjustedMag;
    tempAdjustedValues[1] = Math.cos(rawAngle) * adjustedMag;
    tempAdjustedValues[2] = expoScaleYaw.apply(rawYaw);
    return tempAdjustedValues;
  }

  @Override
  public void execute() {
    adjustedValues =
        calcAdjustedValues(
            joystick.getRawAxis(RobotContainer.Axis.LEFT_X.id),
            joystick.getRawAxis(RobotContainer.Axis.LEFT_Y.id),
            joystick.getRawAxis(RobotContainer.Axis.RIGHT_Y.id));

    driveSubsystem.drive(
        DriveConstants.kMaxSpeedMetersPerSecond * -adjustedValues[0],
        DriveConstants.kMaxSpeedMetersPerSecond * -adjustedValues[1],
        DriveConstants.kMaxOmega * -adjustedValues[2]);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0, 0, 0);
  }
}
