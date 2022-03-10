package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.auton.TwoCargoAuto;
import frc.robot.commands.drive.DriveAutonCommand;
import java.util.ArrayList;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.thirdcoast.util.AutonSwitch;

public class AutoSwitch {
  private final AutonSwitch autonSwitch;
  private ArrayList<DigitalInput> switchInputs = new ArrayList<>();
  private int currAutoSwitchPos = -1;
  private int newAutoSwitchPos;
  private int autoSwitchStableCounts = 0;
  private Logger logger = LoggerFactory.getLogger(AutoSwitch.class);
  private Command autoCommand;
  private final DriveSubsystem driveSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final MagazineSubsystem magazineSubsystem;
  private final TurretSubsystem turretSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final VisionSubsystem visionSubsystem;

  public AutoSwitch(
      DriveSubsystem driveSubsystem,
      IntakeSubsystem intakeSubsystem,
      MagazineSubsystem magazineSubsystem,
      TurretSubsystem turretSubsystem,
      ShooterSubsystem shooterSubsystem,
      VisionSubsystem visionSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.magazineSubsystem = magazineSubsystem;
    this.turretSubsystem = turretSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.visionSubsystem = visionSubsystem;

    for (int i = AutoConstants.kStartSwitchId; i <= AutoConstants.kEndSwitchId; i++) {
      switchInputs.add(new DigitalInput(i));
    }
    autonSwitch = new AutonSwitch(switchInputs);
  }

  public void checkSwitch() {
    if (hasSwitchChanged()) {
      logger.info(
          "Initializing Auto Switch Position: {}", String.format("%02X", currAutoSwitchPos));
      autoCommand = getAutoCommand(currAutoSwitchPos);
    }
  }

  public void resetSwitch() {
    if (currAutoSwitchPos == -1) {
      logger.info("Reset Auton Switch");
    }
    currAutoSwitchPos = -1;
  }

  public Command getAutoCommand() {
    return autoCommand;
  }

  private boolean hasSwitchChanged() {
    boolean changed = false;
    int switchPos = autonSwitch.position();

    if (switchPos != newAutoSwitchPos) {
      autoSwitchStableCounts = 0;
      newAutoSwitchPos = switchPos;
    } else autoSwitchStableCounts++;

    if (autoSwitchStableCounts > AutoConstants.kStableCounts
        && currAutoSwitchPos != newAutoSwitchPos) {
      changed = true;
      currAutoSwitchPos = newAutoSwitchPos;
    }
    return changed;
  }

  private Command getAutoCommand(int switchPos) {
    switch (switchPos) {
      case 0x10:
        return new TwoCargoAuto(
            visionSubsystem,
            turretSubsystem,
            shooterSubsystem,
            magazineSubsystem,
            intakeSubsystem,
            driveSubsystem,
            "LeftCargo1Collect",
            AutoConstants.kLeftStartYaw);
      case 0x20:
        return new TwoCargoAuto(
            visionSubsystem,
            turretSubsystem,
            shooterSubsystem,
            magazineSubsystem,
            intakeSubsystem,
            driveSubsystem,
            "MidCargo1Collect",
            AutoConstants.kMidStartYaw);
      case 0x30:
        return new TwoCargoAuto(
            visionSubsystem,
            turretSubsystem,
            shooterSubsystem,
            magazineSubsystem,
            intakeSubsystem,
            driveSubsystem,
            "RightCargo1Collect",
            AutoConstants.kRightStartYaw);
      default:
        String msg = String.format("no auto command assigned for switch position %02X", switchPos);
        DriverStation.reportWarning(msg, false);
        return new DriveAutonCommand(driveSubsystem, "DefaultPath");
    }
  }
}