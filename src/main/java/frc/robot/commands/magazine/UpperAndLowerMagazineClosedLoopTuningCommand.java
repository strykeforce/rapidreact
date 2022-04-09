package frc.robot.commands.magazine;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.DashboardConstants;
import frc.robot.Constants.MagazineConstants;
import frc.robot.subsystems.MagazineSubsystem;

public class UpperAndLowerMagazineClosedLoopTuningCommand extends InstantCommand {
  public final MagazineSubsystem magazineSubsystem;

  public UpperAndLowerMagazineClosedLoopTuningCommand(MagazineSubsystem magazineSubsystem) {
    addRequirements(magazineSubsystem);
    this.magazineSubsystem = magazineSubsystem;
  }

  @Override
  public void initialize() {
    magazineSubsystem.enableUpperBeamBreak(false);
    magazineSubsystem.enableLowerBeamBreak(false);
    magazineSubsystem.manualClosedLoopFullMagazine(
        MagazineConstants.kLowerMagazineIntakeSpeed,
        SmartDashboard.getNumber(
            DashboardConstants.kTuneUpperMagSpeedTicks,
            MagazineConstants.kUpperMagazineIntakeSpeed));
  }
}
