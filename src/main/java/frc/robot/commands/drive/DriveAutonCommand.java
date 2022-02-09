package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PathData;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class DriveAutonCommand extends CommandBase {
  private final DriveSubsystem driveSubsystem;
  private final Trajectory trajectory;
  private final Timer timer = new Timer();
  private static final Logger logger = LoggerFactory.getLogger(DriveAutonCommand.class);
  private final Rotation2d robotHeading;

  public DriveAutonCommand(DriveSubsystem driveSubsystem, String trajectoryName) {
    addRequirements(driveSubsystem);
    this.driveSubsystem = driveSubsystem;
    PathData pathdata = driveSubsystem.generateTrajectory(trajectoryName);
    trajectory = pathdata.trajectory;
    robotHeading = pathdata.targetYaw;
    timer.start();
  }

  @Override
  public void initialize() {
    driveSubsystem.setEnableHolo(true);
    Pose2d initialPose = trajectory.getInitialPose();
    driveSubsystem.resetOdometry(
        new Pose2d(initialPose.getTranslation(), driveSubsystem.getGyroRotation2d()));
    driveSubsystem.grapherTrajectoryActive(true);
    timer.reset();
    logger.info("Begin Trajectory");
  }

  @Override
  public void execute() {
    Trajectory.State desiredState = trajectory.sample(timer.get());
    driveSubsystem.calculateController(desiredState, robotHeading);
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(trajectory.getTotalTimeSeconds());
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.setEnableHolo(false);
    driveSubsystem.calculateController(
        trajectory.sample(trajectory.getTotalTimeSeconds()), robotHeading);
    driveSubsystem.grapherTrajectoryActive(false);
    logger.info("End Trajectory: {}", timer.get());
  }
}