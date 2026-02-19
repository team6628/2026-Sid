package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
 * Command-based wrapper for CTRE Tuner Swerve Drivetrain
 * Configured for ROBOT-CENTRIC driving
 */
public class CommandSwerveDrivetrain
        extends frc.robot.generated.TunerConstants.TunerSwerveDrivetrain
        implements Subsystem {

    private static final double kSimLoopPeriod = 0.005;

    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private final SysIdRoutine m_sysIdRoutine;

    /* ============================= */
    /* ===== CONSTRUCTORS ========= */
    /* ============================= */

    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {

        super(drivetrainConstants, modules);

        m_sysIdRoutine = createSysIdRoutine();

        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules) {

        super(drivetrainConstants, odometryUpdateFrequency, modules);

        m_sysIdRoutine = createSysIdRoutine();

        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

      // =========================
    // APPLY REQUEST METHOD
    // =========================
    public Command applyRequest(
        java.util.function.Supplier<SwerveRequest> requestSupplier) {

        return run(() -> this.setControl(requestSupplier.get()));
    }


    /* ============================= */
    /* ===== SYSID ================= */
    /* ============================= */

    private SysIdRoutine createSysIdRoutine() {
        return new SysIdRoutine(
                new SysIdRoutine.Config(
                        null,
                        Volts.of(4),
                        null,
                        (state) -> DriverStation.isAutonomous()),
                new SysIdRoutine.Mechanism(
                        (volts) -> this.setControl(
                                new SwerveRequest.SysIdSwerveTranslation()
                                        .withVolts(volts)),
                        null,
                        this));
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }

    /* ============================= */
    /* ===== ROBOT-CENTRIC DRIVE === */
    /* ============================= */

    /**
     * Applies a robot-centric control request.
     */
    public Command applyRobotCentric(
            Supplier<Double> xSpeed,
            Supplier<Double> ySpeed,
            Supplier<Double> rotSpeed) {

        return run(() -> this.setControl(
                new SwerveRequest.RobotCentric()
                        .withVelocityX(xSpeed.get())
                        .withVelocityY(ySpeed.get())
                        .withRotationalRate(rotSpeed.get())
        ));
    }

    /* ============================= */
    /* ===== SEED GYRO COMMANDS ==== */
    /* ============================= */

    public Command seedFieldCentricCommand() {
        return runOnce(() -> super.seedFieldCentric());
    }

    public Command seedFieldCentricCommand(Rotation2d rotation) {
        return runOnce(() -> super.seedFieldCentric(rotation));
    }

    /* ============================= */
    /* ===== SIM SUPPORT =========== */
    /* ============================= */

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        m_simNotifier = new Notifier(() -> {
            double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });

        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    @Override
    public void periodic() {
        // Called by CommandScheduler
    }
}