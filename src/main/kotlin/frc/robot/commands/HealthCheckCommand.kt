package frc.robot.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.DriveSubsystem
import frc.robot.subsystems.MagazineSubsystem
import frc.robot.subsystems.IntakeSubsystem
import frc.robot.subsystems.ShooterSubsystem
import frc.robot.subsystems.TurretSubsystem
import mu.KotlinLogging
import org.strykeforce.healthcheck.HealthCheck
import org.strykeforce.healthcheck.healthCheck
import org.strykeforce.swerve.TalonSwerveModule

private val logger = KotlinLogging.logger {}

class HealthCheckCommand(val driveSubsystem : DriveSubsystem, 
                        val magazineSubsystem : MagazineSubsystem, 
                        val intakeSubsystem : IntakeSubsystem, 
                        val shooterSubsystem : ShooterSubsystem, 
                        val turretSubsystem : TurretSubsystem) : CommandBase() {
    init {
        addRequirements(
            driveSubsystem,
            magazineSubsystem,
            intakeSubsystem,
            shooterSubsystem,
            turretSubsystem
        )
        magazineSubsystem.setManualState()
        shooterSubsystem.setManualState()
    }

    private lateinit var healthCheck : HealthCheck

    override fun initialize() {
        healthCheck = healthCheck {
            // azimuths
            talonCheck {
                name = "Swerve Azimuth Tests"
                talons = driveSubsystem.swerveModules.map {
                    val module = it as? TalonSwerveModule ?: throw IllegalStateException()
                    module.azimuthTalon
                }

                val volt3supplyCurrentRange = 0.25..0.75
                val volt6supplyCurrentRange = 0.5..1.25
                val volt9supplyCurrentRange = 1.0..1.5

                val volt3statorCurrentRange = 0.25..0.75
                val volt6statorCurrentRange = 0.5..1.25
                val volt9statorCurrentRange = 1.0..1.5

                timedTest {
                    percentOutput = 0.25
                    supplyCurrentRange = volt3supplyCurrentRange
                    statorCurrentRange = volt3statorCurrentRange
                    speedRange = 230..260
                }

                timedTest {
                    percentOutput = -0.25
                    supplyCurrentRange = volt3supplyCurrentRange
                    statorCurrentRange = volt3statorCurrentRange
                    speedRange = -260..-230
                }

                timedTest {
                    percentOutput = 0.5
                    supplyCurrentRange = volt6supplyCurrentRange
                    statorCurrentRange = volt6statorCurrentRange
                    speedRange = 510..550
                }

                timedTest {
                    percentOutput = -0.5
                    supplyCurrentRange = volt6supplyCurrentRange
                    statorCurrentRange = volt6statorCurrentRange
                    speedRange = -550..-510
                }

                timedTest {
                    percentOutput = 0.75
                    supplyCurrentRange = volt9supplyCurrentRange
                    statorCurrentRange = volt9statorCurrentRange
                    speedRange = 780..840
                }

                timedTest {
                    percentOutput = -0.75
                    supplyCurrentRange = volt9supplyCurrentRange
                    statorCurrentRange = volt9statorCurrentRange
                    speedRange = -840..-780
                }
            }
            // drive
            talonCheck {
                name = "Swerve Drive Tests"
                talons = driveSubsystem.swerveModules.map {
                    val module = it as? TalonSwerveModule ?: throw IllegalStateException()
                    module.driveTalon
                }

                val volt3supplyCurrentRange = 0.25..0.75
                val volt6supplyCurrentRange = 0.5..1.1
                val volt9supplyCurrentRange = 3.5..4.75

                val volt3statorCurrentRange = 0.5..1.125
                val volt6statorCurrentRange = 1.0..2.0
                val volt9statorCurrentRange = 2.5..5.0

                timedTest {
                    percentOutput = 0.25
                    supplyCurrentRange = volt3supplyCurrentRange
                    statorCurrentRange = volt3statorCurrentRange
                    speedRange = 5425..5575
                }

                timedTest {
                    percentOutput = -0.25
                    supplyCurrentRange = volt3supplyCurrentRange
                    statorCurrentRange = volt3statorCurrentRange
                    speedRange = -5575..-5425
                }

                timedTest {
                    percentOutput = 0.5
                    supplyCurrentRange = volt6supplyCurrentRange
                    statorCurrentRange = volt6statorCurrentRange
                    speedRange = 10925..11150
                }

                timedTest {
                    percentOutput = -0.5
                    supplyCurrentRange = volt6supplyCurrentRange
                    statorCurrentRange = volt6statorCurrentRange
                    speedRange = -11150..-10925
                }

                timedTest {
                    percentOutput = 1.0
                    supplyCurrentRange = volt9supplyCurrentRange
                    statorCurrentRange = volt9statorCurrentRange
                    speedRange = 21350..21800
                }

                timedTest {
                    percentOutput = -1.0
                    supplyCurrentRange = volt9supplyCurrentRange
                    statorCurrentRange = volt9statorCurrentRange
                    speedRange = -21800..-21350
                }
            }
            // intake
            talonCheck {
                name = "Intake Tests"
                talons = intakeSubsystem.talons
                
                val volt6supplyCurrentRange = 1.0..4.0

                val volt6statorCurrentRange = 1.0..4.0

                timedTest {
                    percentOutput = 0.5
                    supplyCurrentRange = volt6supplyCurrentRange
                    statorCurrentRange = volt6statorCurrentRange
                    speedRange = 3500..4500
                }

                timedTest {
                    percentOutput = -0.5
                    supplyCurrentRange = volt6supplyCurrentRange
                    statorCurrentRange = volt6statorCurrentRange
                    speedRange = -4500..-3500
                }
            }
            // magazine
            talonCheck {
                name = "Magazine Tests"
                talons = magazineSubsystem.talons

                val volt6supplyCurrentRange = 1.0..4.0
                val volt12supplyCurrentRange = 13.0..17.0

                val volt6statorCurrentRange = 1.0..4.0
                val volt12statorCurrentRange = 13.0..17.0

                timedTest {
                    percentOutput = 0.5
                    supplyCurrentRange = volt6supplyCurrentRange
                    statorCurrentRange = volt6statorCurrentRange
                    speedRange = 2000..3000
                }

                timedTest {
                    percentOutput = -0.5
                    supplyCurrentRange = volt6supplyCurrentRange
                    statorCurrentRange = volt6statorCurrentRange
                    speedRange = -3000..-2000
                }

                timedTest {
                    percentOutput = 0.8
                    supplyCurrentRange = volt12supplyCurrentRange
                    statorCurrentRange = volt12statorCurrentRange
                    speedRange = 3500..5000
                }

                timedTest {
                    percentOutput = -0.8
                    supplyCurrentRange = volt12supplyCurrentRange
                    statorCurrentRange = volt12statorCurrentRange
                    speedRange = -5000..-3500
                }
            }
            // turret
            talonCheck {
                name = "Turret Tests"
                talons = turretSubsystem.talons

                val supplyCurrentRange = 0.375..1.0
                val statorCurrentRange = 0.375..1.0

                positionTalon {
                    encoderTarget = -10_000
                    encoderGoodEnough = 100
                }

                positionTest {
                    percentOutput = 0.3
                    encoderChangeTarget = 20_000
                    encoderGoodEnough = 500
                    encoderTimeCount = 500

                    supplyCurrentRange
                    statorCurrentRange
                    speedRange = 550..750
                }

                positionTest {
                    percentOutput = -0.3
                    encoderChangeTarget = 20_000
                    encoderGoodEnough = 500
                    encoderTimeCount = 500

                    supplyCurrentRange
                    statorCurrentRange
                    speedRange = -750..-550
                }
                positionTalon {
                    encoderTarget = 0
                    encoderGoodEnough = 100
                }
            }
            // shooter
            talonCheck {
                name = "Shooter Tests"
                talons = shooterSubsystem.shooterTalons

                val volt10_000supplyCurrentRange = 1.0..4.0
                val volt16_000supplyCurrentRange = 13.0..17.0

                val volt10_000statorCurrentRange = 1.0..4.0
                val volt16_000statorCurrentRange = 13.0..17.0

                timedTest {
                    percentOutput = 0.53
                    supplyCurrentRange = volt10_000supplyCurrentRange
                    statorCurrentRange = volt10_000statorCurrentRange
                    speedRange = 8_000..12_000
                }

                timedTest {
                    percentOutput = 0.84
                    supplyCurrentRange = volt16_000supplyCurrentRange
                    statorCurrentRange = volt16_000statorCurrentRange
                    speedRange = 14_000..19_000
                }
            }
            // hood
            talonCheck {
                name = "Hood Tests"
                talons = shooterSubsystem.hoodTalons

                val supplyCurrent = 0.375..1.0
                val statorCurrent = 0.375..1.0

                positionTalon {
                    encoderTarget = 0
                    encoderGoodEnough = 100
                }

                positionTest {
                    percentOutput = 0.2
                    encoderChangeTarget = 5000
                    encoderGoodEnough = 500
                    encoderTimeCount = 500

                    supplyCurrentRange = supplyCurrent
                    statorCurrentRange = statorCurrent
                }

                positionTest {
                    percentOutput = -0.2
                    encoderChangeTarget = 5000
                    encoderGoodEnough = 500
                    encoderTimeCount = 500

                    supplyCurrentRange = supplyCurrent
                    statorCurrentRange = statorCurrent
                }
            }
        }
    }

    override fun execute() {
        healthCheck.execute()
    }

    override fun isFinished() = healthCheck.isFinished()

    override fun end(interrupted: Boolean) {
        healthCheck.report()
        driveSubsystem.lockZero()
        turretSubsystem.rotateTo(0.0)
        shooterSubsystem.hoodClosedLoop(100.0)
    }
}