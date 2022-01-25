package frc.robot.subsystems;

import java.util.Set;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class ShooterSubsystem extends MeasurableSubsystem {

    private static final Logger logger = LoggerFactory.getLogger(ShooterSubsytem.class);

    public ShooterSubsytem(){

    }

    public void shooterOpenLoop(double speed){
        logger.info("Shooter on {}", speed);
    }

    public void hoodOpenLoop(double speed){
        logger.info("hood on {}", speed);
    }

    @Override
    public Set<Measure> getMeasures() {
        return Set.of();
    }
}