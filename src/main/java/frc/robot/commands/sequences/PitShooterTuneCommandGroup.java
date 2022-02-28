package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.MagazineConstants;
import frc.robot.commands.intake.IntakeOpenLoopCommand;
import frc.robot.commands.magazine.LowerMagazineOpenLoopCommand;
import frc.robot.commands.magazine.UpperMagazineOpenLoopCommand;
import frc.robot.commands.shooter.PitHoodClosedLoopCommand;
import frc.robot.commands.shooter.PitShooterClosedLoopCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class PitShooterTuneCommandGroup extends ParallelCommandGroup {
  public PitShooterTuneCommandGroup(
      ShooterSubsystem shooterSubsystem,
      MagazineSubsystem magazineSubsystem,
      IntakeSubsystem intakeSubsystem) {
    addCommands(
        new PitShooterClosedLoopCommand(shooterSubsystem),
        new PitHoodClosedLoopCommand(shooterSubsystem),
        new IntakeOpenLoopCommand(intakeSubsystem, IntakeConstants.kIntakeSpeed),
        new LowerMagazineOpenLoopCommand(magazineSubsystem, MagazineConstants.kMagazineIntakeSpeed),
        new UpperMagazineOpenLoopCommand(
            magazineSubsystem, MagazineConstants.kMagazineIntakeSpeed));
  }
}
