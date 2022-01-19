package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveTeleopCommand extends CommandBase {
  private Joystick joystick;
  private DriveSubsystem driveSubsystem;

  public DriveTeleopCommand(Joystick driver, DriveSubsystem driveSubsystem) {
    addRequirements(driveSubsystem);
    joystick = driver;
    this.driveSubsystem = driveSubsystem;
  }

  @Override
  public void execute() {
    driveSubsystem.drive(
        deadband(joystick.getRawAxis(Axis.LEFT_X.id)),
        deadband(joystick.getRawAxis(Axis.LEFT_Y.id)),
        deadband(joystick.getRawAxis(Axis.RIGHT_Y.id)));
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
    } else {
      return stickValue;
    }
  }

  private enum Axis {
    RIGHT_X(1),
    RIGHT_Y(0),
    LEFT_X(2),
    LEFT_Y(5),
    TUNER(6),
    LEFT_BACK(4),
    RIGHT_BACK(3);

    private final int id;

    Axis(int id) {
      this.id = id;
    }
  }
}
