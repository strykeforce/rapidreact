package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.auton.DefenseAuto;
import frc.robot.commands.auton.DefenseTwoAuto;
import frc.robot.commands.auton.FiveCargoAuto;
import frc.robot.commands.auton.ThreeCargoAuto;
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
  private final IntakeExtendSubsystem intakeExtendSubsystem;
  private final MagazineSubsystem magazineSubsystem;
  private final TurretSubsystem turretSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final VisionSubsystem visionSubsystem;

  public AutoSwitch(
      DriveSubsystem driveSubsystem,
      IntakeSubsystem intakeSubsystem,
      IntakeExtendSubsystem intakeExtendSubsystem,
      MagazineSubsystem magazineSubsystem,
      TurretSubsystem turretSubsystem,
      ShooterSubsystem shooterSubsystem,
      VisionSubsystem visionSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.intakeExtendSubsystem = intakeExtendSubsystem;
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
            intakeExtendSubsystem,
            driveSubsystem,
            "LeftCargo1Collect",
            AutoConstants.kLeftStartYaw,
            0.0,
            230.0); // FIXME
      case 0x11:
        return new TwoCargoAuto(
            visionSubsystem,
            turretSubsystem,
            shooterSubsystem,
            magazineSubsystem,
            intakeSubsystem,
            intakeExtendSubsystem,
            driveSubsystem,
            "LeftCargo1Collect",
            AutoConstants.kLeftStartYaw,
            3.0,
            230.0); // FIXME
      case 0x12:
        return new DefenseAuto(
            visionSubsystem,
            turretSubsystem,
            shooterSubsystem,
            magazineSubsystem,
            intakeSubsystem,
            intakeExtendSubsystem,
            driveSubsystem,
            "LeftCargo1Collect",
            "OppCargo2Collect",
            "OppCargo3Collect",
            "DefenseMoveOffOppBall",
            AutoConstants.kLeftStartYaw,
            0.0,
            230.0); // FIXME
      case 0x20:
        return new TwoCargoAuto(
            visionSubsystem,
            turretSubsystem,
            shooterSubsystem,
            magazineSubsystem,
            intakeSubsystem,
            intakeExtendSubsystem,
            driveSubsystem,
            "MidCargo1Collect",
            AutoConstants.kMidStartYaw,
            0.0,
            226.0); // FIXME
      case 0x21:
        return new TwoCargoAuto(
            visionSubsystem,
            turretSubsystem,
            shooterSubsystem,
            magazineSubsystem,
            intakeSubsystem,
            intakeExtendSubsystem,
            driveSubsystem,
            "MidCargo1Collect",
            AutoConstants.kMidStartYaw,
            3.0,
            226.0); // FIXME
      case 0x30:
        return new TwoCargoAuto(
            visionSubsystem,
            turretSubsystem,
            shooterSubsystem,
            magazineSubsystem,
            intakeSubsystem,
            intakeExtendSubsystem,
            driveSubsystem,
            "RightCargo1Collect",
            AutoConstants.kRightStartYaw,
            0.0,
            248.0); // FIXME
      case 0x31:
        return new TwoCargoAuto(
            visionSubsystem,
            turretSubsystem,
            shooterSubsystem,
            magazineSubsystem,
            intakeSubsystem,
            intakeExtendSubsystem,
            driveSubsystem,
            "RightCargo1Collect",
            AutoConstants.kRightStartYaw,
            3.0,
            248.0); // FIXME
      case 0x32:
        return new ThreeCargoAuto(
            visionSubsystem,
            turretSubsystem,
            shooterSubsystem,
            magazineSubsystem,
            intakeSubsystem,
            intakeExtendSubsystem,
            driveSubsystem,
            "RightCargo1Collect",
            "RightCargo2Collect",
            AutoConstants.kRightStartYaw,
            0.0,
            248.0,
            247.0); // FIXME
      case 0x34:
        return new FiveCargoAuto(
            visionSubsystem,
            turretSubsystem,
            shooterSubsystem,
            magazineSubsystem,
            intakeSubsystem,
            intakeExtendSubsystem,
            driveSubsystem,
            "RightCargo1Collect",
            "RightCargo2Collect",
            "RightCargo3Collect",
            "RightCargo3Shoot",
            AutoConstants.kRightStartYaw,
            0.0,
            248.0,
            247.0,
            138.0);
      case 0x35:
        return new FiveCargoAuto(
            visionSubsystem,
            turretSubsystem,
            shooterSubsystem,
            magazineSubsystem,
            intakeSubsystem,
            intakeExtendSubsystem,
            driveSubsystem,
            "RightCargo1Collect",
            "BlueRightCargo2Collect",
            "BlueRightCargo3Collect",
            "BlueRightCargo3Shoot",
            AutoConstants.kRightStartYaw,
            0.0,
            248.0,
            247.0,
            138.0);
      case 0x45:
        return new DefenseTwoAuto(visionSubsystem, turretSubsystem, shooterSubsystem, magazineSubsystem, intakeSubsystem, intakeExtendSubsystem, driveSubsystem, "", "", AutoConstants.kDefenseTwoRightStartYaw, 5.0, 200);
      default:
        String msg = String.format("no auto command assigned for switch position %02X", switchPos);
        DriverStation.reportWarning(msg, false);
        return new DriveAutonCommand(driveSubsystem, "DefaultPath", true, false);
    }
  }
}
