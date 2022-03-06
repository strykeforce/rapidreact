package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotContainer.Axis;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class DriveClimbCommand extends CommandBase {
  private final DriveSubsystem driveSubsystem;
  private final Joystick driveJoystick;
  private final ClimbSubsystem climbSubsystem;
  private static final Logger logger = LoggerFactory.getLogger(DriveClimbCommand.class);

  public DriveClimbCommand(
      DriveSubsystem driveSubsystem, Joystick driveJoystick, ClimbSubsystem climbSubsystem) {
    addRequirements(driveSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.driveJoystick = driveJoystick;
    this.climbSubsystem = climbSubsystem;
  }

  @Override
  public void execute() {
    double fwd =
        MathUtil.applyDeadband(
            driveJoystick.getRawAxis(Axis.LEFT_X.id), DriveConstants.kDeadbandAllStick);
    double str =
        MathUtil.applyDeadband(
            driveJoystick.getRawAxis(Axis.LEFT_Y.id), DriveConstants.kDeadbandAllStick);
    double yaw =
        MathUtil.applyDeadband(
            driveJoystick.getRawAxis(Axis.RIGHT_Y.id), DriveConstants.kDeadbandAllStick);

    // Determine Jack Factor
    double addedYaw = 0.0;
    if (climbSubsystem.isRightArmTouchingBar() && !climbSubsystem.isLeftArmTouchingBar()) {
      addedYaw = DriveConstants.kYawJackFactorClimb;
      logger.info("Right Arm Touching Bar, Left Arm not Touching Bar yaw: {} + {} ", yaw, addedYaw);
    } else if (climbSubsystem.isLeftArmTouchingBar() && !climbSubsystem.isRightArmTouchingBar()) {
      addedYaw = -DriveConstants.kYawJackFactorClimb;
      logger.info("Left Arm Touching Bar, Right Arm not Touching Bar yaw: {} + {}", yaw, addedYaw);
    }

    driveSubsystem.drive(
        fwd * DriveConstants.kMaxFwdStrStickClimb,
        str * DriveConstants.kMaxFwdStrStickClimb,
        (yaw + addedYaw) * -DriveConstants.kMaxYawStickClimb);
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
