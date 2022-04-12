package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeExtendSubsystem;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.MagazineSubsystem.LowerMagazineState;

public class RetractOnFullCommand extends CommandBase {
  private final MagazineSubsystem magazineSubsystem;
  private final IntakeExtendSubsystem intakeExtendSubsystem;

  public RetractOnFullCommand(
      MagazineSubsystem magazineSubsystem, IntakeExtendSubsystem intakeExtendSubsystem) {
    addRequirements(intakeExtendSubsystem);
    this.magazineSubsystem = magazineSubsystem;
    this.intakeExtendSubsystem = intakeExtendSubsystem;
  }

  @Override
  public boolean isFinished() {
    return magazineSubsystem.getCurrLowerMagazineState() == LowerMagazineState.WAIT_UPPER
        && magazineSubsystem.isMagazineFull();
  }

  @Override
  public void end(boolean interrupted) {
    intakeExtendSubsystem.retractClosedLoop();
  }
}
