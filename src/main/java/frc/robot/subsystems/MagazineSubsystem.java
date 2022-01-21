package frc.robot.subsystems;

import java.security.acl.LastOwnerException;
import java.util.Set;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.ColorSensorV3;

import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.robot.Constants.MagazineConstants;

public class MagazineSubsystem extends MeasurableSubsystem {
    private ColorSensorV3 colorSensor;
    private Color lastColor=new Color(0, 0, 0);
    private TalonSRX magazineTalon;

    public MagazineSubsystem() {
        colorSensor=new ColorSensorV3 (Port.kMXP);
        magazineTalon=new TalonSRX (MagazineConstants.MagazineTalonID);
        magazineTalon.configFactoryDefault(Constants.kTalonConfigTimeout);
        magazineTalon.configAllSettings(MagazineConstants.getMagazineTalonConfig(), Constants.kTalonConfigTimeout);
        magazineTalon.enableCurrentLimit(true);
        magazineTalon.enableVoltageCompensation(true);
        magazineTalon.setNeutralMode(NeutralMode.Coast);
    }

    public void openLoopRotate(double percentOutput) {
        magazineTalon.set(ControlMode.PercentOutput, percentOutput);
    }

    public Color getColor() {
        lastColor=colorSensor.getColor();
        return lastColor;
    }

    @Override
    public void registerWith(TelemetryService telemetryService) {
        super.registerWith(telemetryService);
        telemetryService.register(magazineTalon);
    }

    @Override
    public Set<Measure> getMeasures() {
        return Set.of(
            new Measure ("red", () -> lastColor.red),
            new Measure ("blue", () -> lastColor.blue),
            new Measure ("green", () -> lastColor.green)
        );

    }

    
}
