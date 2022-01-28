package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.intake.IntakeOpenLoopCommand;
import frc.robot.commands.magazine.WaitForFirstCargoColorCommand;
import frc.robot.commands.magazine.WaitForSecondCargoColorCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MagazineSubsystem;

public class AutoIntakeCommandGroup extends ParallelDeadlineGroup {
    private IntakeSubsystem intakeSubystem;
    private MagazineSubsystem magazineSubsystem;

    public AutoIntakeCommandGroup (IntakeSubsystem intakeSubsystem, MagazineSubsystem magazineSubsystem) {
        setDeadline(
            new WaitForSecondCargoColorCommand(magazineSubsystem)

        );
        addCommands(new IntakeOpenLoopCommand);
    }
}
