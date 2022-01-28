package frc.robot.commands.magazine;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.MagazineSubsystem.CargoColor;

public class WaitForFirstCargoColorCommand extends CommandBase {
    private MagazineSubsystem magazineSubsystem;

    public WaitForFirstCargoColorCommand(MagazineSubsystem magazineSubsystem) {
        addRequirements(magazineSubsystem);
        this.magazineSubsystem = magazineSubsystem;
    }

    @Override
    public boolean isFinished() {
        CargoColor knownCargoColor = magazineSubsystem.readCargoColor();
        if (knownCargoColor == CargoColor.NONE) return false;
        else return true;
    }   
}
