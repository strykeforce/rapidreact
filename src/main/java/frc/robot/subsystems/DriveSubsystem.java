package frc.robot.subsystems;

import org.strykeforce.swerve.SwerveDrive;
import org.strykeforce.swerve.TalonSwerveModule;
import org.strykeforce.telemetry.TelemetryService;

import frc.robot.Registerable;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import static frc.robot.Constants.kTalonConfigTimeout;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class DriveSubsystem extends SubsystemBase implements Registerable {

    private SwerveDrive swerveDrive;
    public DriveSubsystem()
    {
        
            var moduleBuilder =
                new TalonSwerveModule.Builder()
                    .driveGearRatio(DriveConstants.kDriveGearRatio)
                    .wheelDiameterInches(DriveConstants.kWheelDiameterInches)
                    .driveMaximumMetersPerSecond(DriveConstants.kMaxSpeedMetersPerSecond);
        
            TalonSwerveModule[] swerveModules = new TalonSwerveModule[4];
            Translation2d[] wheelLocations = DriveConstants.getWheelLocationMeters();
        
           
        
            for (int i = 0; i < 4; i++) {
              var azimuthTalon = new TalonSRX(i);
              azimuthTalon.configFactoryDefault(kTalonConfigTimeout);
              azimuthTalon.configAllSettings(DriveConstants.getAzimuthTalonConfig(), kTalonConfigTimeout);
              azimuthTalon.enableCurrentLimit(true);
              azimuthTalon.enableVoltageCompensation(true);
              azimuthTalon.setNeutralMode(NeutralMode.Coast);
        
              var driveTalon = new TalonFX(i + 10);
              driveTalon.configFactoryDefault(kTalonConfigTimeout);
              driveTalon.configAllSettings(DriveConstants.getDriveTalonConfig(), kTalonConfigTimeout);
              driveTalon.enableVoltageCompensation(true);
              driveTalon.setNeutralMode(NeutralMode.Brake);
        
              swerveModules[i] =
                  moduleBuilder
                      .azimuthTalon(azimuthTalon)
                      .driveTalon(driveTalon)
                      .wheelLocationMeters(wheelLocations[i])
                      .build();
        
              swerveModules[i].loadAndSetAzimuthZeroReference();
              
              
            }
        
            swerveDrive = new SwerveDrive(swerveModules);
            swerveDrive.resetGyro();
            swerveDrive.setGyroOffset(Rotation2d.fromDegrees(180));
        
    }

    @Override
    public void registerTelemetry(TelemetryService telemetryService) {
    }

    public void drive(double forwardMetersPerSec, double strafeMetersPerSec, double yawRadiansPerSec)
    {
        swerveDrive.drive(forwardMetersPerSec, strafeMetersPerSec, yawRadiansPerSec, true);
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        swerveDrive.periodic();
    }

    public void resetGyro()
    {
        swerveDrive.resetGyro();
    }
}
