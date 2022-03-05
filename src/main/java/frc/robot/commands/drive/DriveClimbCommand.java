package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotContainer.Axis;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class DriveClimbCommand extends CommandBase {
  private final DriveSubsystem driveSubsystem;
  private final Joystick driveJoystick;
  private final ClimbSubsystem climbSubsystem;

  public DriveClimbCommand(
      DriveSubsystem driveSubsystem, Joystick driveJoystick, ClimbSubsystem climbSubsystem) {
    addRequirements(driveSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.driveJoystick = driveJoystick;
    this.climbSubsystem = climbSubsystem;
  }

  @Override
  public void execute() {
    // Determine Jack Factor
    double addedYaw = 0.0;
    if (climbSubsystem.isRightArmTouchingBar() && !climbSubsystem.isLeftArmTouchingBar()) {
      addedYaw = DriveConstants.kYawJackFactorClimb;
    } else if (climbSubsystem.isLeftArmTouchingBar() && !climbSubsystem.isRightArmTouchingBar()) {
      addedYaw = -DriveConstants.kYawJackFactorClimb;
    }

    // Get Deadband
    double fwd =
        MathUtil.applyDeadband(
            driveJoystick.getRawAxis(Axis.LEFT_X.id), DriveConstants.kDeadbandAllStick);
    double str =
        MathUtil.applyDeadband(
            driveJoystick.getRawAxis(Axis.LEFT_Y.id), DriveConstants.kDeadbandAllStick);
    double yaw =
        addedYaw
            + MathUtil.applyDeadband(
                driveJoystick.getRawAxis(Axis.RIGHT_Y.id), DriveConstants.kDeadbandAllStick);

    // Interpolate
    fwd =
        MathUtil.interpolate(
            -DriveConstants.kMaxFwdStrStickClimb, DriveConstants.kMaxFwdStrStickClimb, fwd);
    str =
        MathUtil.interpolate(
            -DriveConstants.kMaxFwdStrStickClimb, DriveConstants.kMaxFwdStrStickClimb, str);
    yaw =
        MathUtil.interpolate(
            -DriveConstants.kMaxYawStickClimb, DriveConstants.kMaxYawStickClimb, yaw);

    driveSubsystem.drive(
        fwd * DriveConstants.kMaxSpeedMetersPerSecond,
        str * DriveConstants.kMaxSpeedMetersPerSecond,
        yaw * -DriveConstants.kMaxOmega);
  }

  @Override
  public boolean isFinished() {
    return climbSubsystem.isRightArmTouchingBar() && climbSubsystem.isLeftArmTouchingBar();
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0.0, 0.0, 0.0);
  }
}
