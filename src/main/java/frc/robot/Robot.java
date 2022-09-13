// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.sequences.SetAllianceCommand;
import frc.robot.commands.vision.DisableVisionCommand;
import frc.robot.commands.vision.EnableVisionCommand;
import frc.robot.commands.vision.TurnOnDeadeyeCommand;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private Logger logger;
  private RobotContainer m_robotContainer;
  private boolean haveAlliance;

  public Robot() {
    // deadeyeNetworkTableInstance = NetworkTableInstance.create();
    // deadeyeNetworkTableInstance.startClient("192.168.3.3", 1736); //TEST DEADEYE
    // deadeyeNetworkTableInstance.startClient("192.168.3.3", 1735); //real one
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    // first call to LoggerFactory.getLogger must be after RobotContainer sets logging location
    logger = LoggerFactory.getLogger(Robot.class);
    haveAlliance = false;

    CommandScheduler.getInstance()
        .schedule(new TurnOnDeadeyeCommand(m_robotContainer.getVisionSubsystem()));

    Shuffleboard.getTab("Match")
        .add("SetAllianceRed", new SetAllianceCommand(Alliance.Red, m_robotContainer))
        .withPosition(2, 0)
        .withSize(1, 1);
    Shuffleboard.getTab("Match")
        .add("SetAllianceBlue", new SetAllianceCommand(Alliance.Blue, m_robotContainer))
        .withPosition(2, 1)
        .withSize(1, 1);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    if (!haveAlliance) {
      Alliance alliance = DriverStation.getAlliance();
      if (alliance != Alliance.Invalid) {
        haveAlliance = true;
        m_robotContainer.setAllianceColor(alliance);
        logger.info("Set Alliance: {}", alliance);
      }
    }
    m_robotContainer.updateMatchData();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    logger.info("Disabled Init");
    CommandScheduler.getInstance()
        .schedule(
            new DisableVisionCommand(m_robotContainer.getVisionSubsystem())
                .beforeStarting(new WaitCommand(1.0)));
  }

  @Override
  public void disabledPeriodic() {
    m_robotContainer.getAutoSwitch().checkSwitch();
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    logger.info("Auto Init");
    m_robotContainer.setDoVisionOdomReset(false);
    Command autoCommand = m_robotContainer.getAutoSwitch().getAutoCommand();
    if (autoCommand != null) {
      autoCommand.schedule();
    }
    m_robotContainer.zeroClimb();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    logger.info("TeleopInit");
    m_robotContainer.zeroClimb();
    m_robotContainer.startAutoIntake();
    m_robotContainer.setDoVisionOdomReset(true);
    CommandScheduler.getInstance()
        .schedule(
            new EnableVisionCommand(
                m_robotContainer.getVisionSubsystem(), m_robotContainer.getDriveSubsystem()));
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.configureManualClimbButtons();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
