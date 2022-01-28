package frc.robot.commands.magazine;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.MagazineSubsystem.CargoColor;

public class WaitForSecondCargoColorCommand extends CommandBase {
    private MagazineSubsystem magazineSubsystem;

    public WaitForSecondCargoColorCommand(MagazineSubsystem magazineSubsystem) {
        addRequirements(magazineSubsystem);
        this.magazineSubsystem = magazineSubsystem;
    }

    @Override
    public boolean isFinished() {
        CargoColor knownCargoColor = magazineSubsystem.readCargoColor();
        CargoColor[] storedCargo = magazineSubsystem.getAllCargoColors();
        if (knownCargoColor != CargoColor.NONE && storedCargo[1] != CargoColor.NONE) return true;
        else return false;
    }
}
