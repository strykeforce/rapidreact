package frc.robot.commands.sequences.shooting;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class UnknownOrderShotAutonCommand extends InstantCommand {
  public final TurretSubsystem turretSubsystem;
  public final ShooterSubsystem shooterSubsystem;
  public final MagazineSubsystem magazineSubsystem;
  public final IntakeSubsystem intakeSubsystem;

  public UnknownOrderShotAutonCommand(
      TurretSubsystem turretSubsystem,
      ShooterSubsystem shooterSubsystem,
      MagazineSubsystem magazineSubsystem,
      IntakeSubsystem intakeSubsystem) {
    addRequirements(turretSubsystem, intakeSubsystem);
    this.turretSubsystem = turretSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.magazineSubsystem = magazineSubsystem;
    this.intakeSubsystem = intakeSubsystem;
  }

  @Override
  public void initialize() {
    if (!magazineSubsystem.isColorSensorIgnored()) {
      if (magazineSubsystem.isFirstCargoAlliance()) {
        turretSubsystem.trackTarget();
        shooterSubsystem.shoot();
        magazineSubsystem.shoot();
      } else {
        turretSubsystem.opponentCargoShot(ShooterConstants.kDestageOpponentCargoShotOdomAimPos);
        shooterSubsystem.geyserShot(true, true, ShooterConstants.kDestageOpponentCargoShotSol);
        magazineSubsystem.shoot();
      }
    }
  }
}
