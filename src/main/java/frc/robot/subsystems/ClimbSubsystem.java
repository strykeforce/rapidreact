package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.Constants;
import frc.robot.Constants.ClimbConstants;
import java.util.Set;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class ClimbSubsystem extends MeasurableSubsystem {
  private final TalonFX extendFalcon1;
  private final TalonFX extendFalcon2;
  private final TalonSRX shoulderFalcon;
  private ClimbState currClimbState = ClimbState.ZEROING;

  public ClimbSubsystem() {
    extendFalcon1 = new TalonFX(Constants.ClimbConstants.kExtend1FalconID);
    extendFalcon2 = new TalonFX(Constants.ClimbConstants.kExtend2FalconID);
    shoulderFalcon = new TalonSRX(Constants.ClimbConstants.kClimbShoulderId);
    configTalons();
  }

  public void rotateShoulder(double speed) {
    shoulderFalcon.set(ControlMode.PercentOutput, speed);
  }

  public void actuateSet1(double speed) {
    extendFalcon1.set(ControlMode.PercentOutput, speed);
  }

  public void actuateSet2(double speed) {
    extendFalcon2.set(ControlMode.PercentOutput, speed);
  }

  public void zeroShoulder() {
    switch (currClimbState) {
      case ZEROING:
        shoulderFalcon.configContinuousCurrentLimit(1, 3);
        shoulderFalcon.set(ControlMode.PercentOutput, -0.2);
        double velocity = shoulderFalcon.getSelectedSensorVelocity();
        if (velocity <= 10) {
          currClimbState = ClimbState.ZEROED;
        }
      case ZEROED:
        {
          shoulderFalcon.configContinuousCurrentLimit(
              Constants.ClimbConstants.getShoulderTalonConfig().continuousCurrentLimit,
              Constants.ClimbConstants.getShoulderTalonConfig().peakCurrentDuration);
          shoulderFalcon.set(ControlMode.PercentOutput, 0);
        }
    }
  }

  private void configTalons() {
    extendFalcon1.configFactoryDefault(Constants.kTalonConfigTimeout);
    extendFalcon1.configAllSettings(
        ClimbConstants.getExtendFalconConfig(), Constants.kTalonConfigTimeout);
    extendFalcon1.enableVoltageCompensation(true);
    extendFalcon1.setNeutralMode(NeutralMode.Brake);

    extendFalcon2.configFactoryDefault(Constants.kTalonConfigTimeout);
    extendFalcon2.configAllSettings(
        ClimbConstants.getExtendFalconConfig(), Constants.kTalonConfigTimeout);
    extendFalcon2.enableVoltageCompensation(true);
    extendFalcon2.setNeutralMode(NeutralMode.Brake);

    shoulderFalcon.configFactoryDefault(Constants.kTalonConfigTimeout);
    shoulderFalcon.configAllSettings(
        ClimbConstants.getShoulderTalonConfig(), Constants.kTalonConfigTimeout);
    shoulderFalcon.enableVoltageCompensation(true);
    shoulderFalcon.setNeutralMode(NeutralMode.Coast);
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    super.registerWith(telemetryService);
    telemetryService.register(extendFalcon1);
    telemetryService.register(extendFalcon2);
    telemetryService.register(shoulderFalcon);
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of();
  }

  public enum ClimbState {
    ZEROING,
    ZEROED
  }
}
