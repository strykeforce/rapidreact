package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class DriveAutonCommand extends CommandBase {
  private DriveSubsystem driveSubsystem;
  private static Trajectory trajectory;
  private static Timer timer = new Timer();
  private static final Logger logger = LoggerFactory.getLogger(DriveSubsystem.class);
  private static Rotation2d robotHeading;

  public DriveAutonCommand(
      DriveSubsystem driveSubsystem, String trajectoryName, Double targetAngle) {
    addRequirements(driveSubsystem);
    this.driveSubsystem = driveSubsystem;
    robotHeading =
        Rotation2d.fromDegrees(targetAngle); // Desired swerve heading through full trajectory
    trajectory = driveSubsystem.generateTrajectory(trajectoryName);
    timer.start();
  }

  @Override
  public void initialize() {
    Pose2d initialPose = trajectory.getInitialPose();
    driveSubsystem.resetOdometry(new Pose2d(initialPose.getTranslation(), robotHeading));
    driveSubsystem.grapherTrajectoryActive(true);
    timer.reset();
    logger.info("Begin Trajectory");
  }

  @Override
  public void execute() {
    Trajectory.State desiredState = new Trajectory.State();
    desiredState = trajectory.sample(timer.get());
    driveSubsystem.calculateController(desiredState, robotHeading);
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(trajectory.getTotalTimeSeconds());
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0, 0, 0);
    driveSubsystem.grapherTrajectoryActive(false);
    logger.info("End Trajectory: {}", timer.get());
  }
}
