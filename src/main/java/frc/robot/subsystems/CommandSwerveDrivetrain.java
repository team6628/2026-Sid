package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.generated.TunerConstants;

import static edu.wpi.first.units.Units.*;

public class CommandSwerveDrivetrain
        extends frc.robot.generated.TunerConstants.TunerSwerveDrivetrain
        implements Subsystem {

    private final Vision vision;

    private static final double kSimLoopPeriod = 0.005;
    private Notifier simNotifier = null;
    private double lastSimTime;

    private final SysIdRoutine sysIdRoutine;

    private final SlewRateLimiter xLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(4);

    /* ========================================================= */
    /* ===================== CONSTRUCTORS ====================== */
    /* ========================================================= */

    // REQUIRED by CTRE + AutoBuilder (DO NOT REMOVE)
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {

        super(drivetrainConstants, modules);

        this.vision = null;

        this.sysIdRoutine = createSysIdRoutine();

        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    // Vision-enabled constructor (used in RobotContainer)
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            Vision vision,
            SwerveModuleConstants<?, ?, ?>... modules) {

        super(drivetrainConstants, modules);

        this.vision = vision;

        this.sysIdRoutine = createSysIdRoutine();

        this.setVisionMeasurementStdDevs(
                VecBuilder.fill(0.7, 0.7, Math.toRadians(10))
        );

        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /* ========================================================= */
    /* ===================== CORE METHODS ====================== */
    /* ========================================================= */

    public Pose2d getPose() {
        return getState().Pose;
    }

    public void resetOdometry(Pose2d pose) {
        resetPose(pose);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return getState().Speeds;
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> setControl(requestSupplier.get()));
    }

    public void stop() {
        setControl(
                new SwerveRequest.RobotCentric()
                        .withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(0)
        );
    }

    public void stopIfAutonomous() {
        if (DriverStation.isAutonomous()) {
            stop();
        }
    }

    /* ========================================================= */
    /* ===================== SYSID ============================= */
    /* ========================================================= */

    private SysIdRoutine createSysIdRoutine() {
        return new SysIdRoutine(
                new SysIdRoutine.Config(null, Volts.of(4), null,
                        state -> DriverStation.isAutonomous()),
                new SysIdRoutine.Mechanism(
                        volts -> setControl(
                                new SwerveRequest.SysIdSwerveTranslation()
                                        .withVolts(volts)
                        ),
                        null,
                        this
                )
        );
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    /* ========================================================= */
    /* ===================== DRIVE CONTROL ===================== */
    /* ========================================================= */

    public Command applyRobotCentric(
            Supplier<Double> xSpeed,
            Supplier<Double> ySpeed,
            Supplier<Double> rotSpeed) {

        return run(() -> {

            setControl(
                    new SwerveRequest.RobotCentric()
                            .withVelocityX(xLimiter.calculate(xSpeed.get()))
                            .withVelocityY(yLimiter.calculate(ySpeed.get()))
                            .withRotationalRate(rotLimiter.calculate(rotSpeed.get()))
            );
        });
    }

    /* ========================================================= */
    /* ===================== VISION FUSION ===================== */
    /* ========================================================= */

    @Override
    public void periodic() {

        // ===== Vision integration =====
        if (vision != null && vision.hasTarget()) {

            var estimate = vision.getPoseEstimate();

            if (estimate.tagCount > 0) {

                if (estimate.tagCount >= 2 || estimate.avgTagDist < 4.0) {

                    addVisionMeasurement(
                            estimate.pose,
                            estimate.timestampSeconds
                    );
                }
            }

            SmartDashboard.putNumber("Vision/TagCount", estimate.tagCount);
            SmartDashboard.putNumber("Vision/AvgDist", estimate.avgTagDist);
        }

        SmartDashboard.putBoolean(
                "Vision/HasTarget",
                vision != null && vision.hasTarget()
        );

        // ===== Module debug =====
        var modules = getModules();

        double wheelCircumference =
                TunerConstants.kWheelRadius.in(Meters) * 2.0 * Math.PI;

        for (int i = 0; i < modules.length; i++) {

            var target = modules[i].getTargetState();
            var actual = modules[i].getCurrentState();

            double actualRPM =
                    (actual.speedMetersPerSecond / wheelCircumference) * 60.0;

            SmartDashboard.putNumber("Swerve/Mod" + i + "/DriveRPM", actualRPM);
            SmartDashboard.putNumber("Swerve/Mod" + i + "/TargetDeg", target.angle.getDegrees());
            SmartDashboard.putNumber("Swerve/Mod" + i + "/ActualDeg", actual.angle.getDegrees());
        }
    }

    /* ========================================================= */
    /* ===================== SIM SUPPORT ======================= */
    /* ========================================================= */

    private void startSimThread() {

        lastSimTime = Utils.getCurrentTimeSeconds();

        simNotifier = new Notifier(() -> {
            double now = Utils.getCurrentTimeSeconds();
            double dt = now - lastSimTime;
            lastSimTime = now;

            updateSimState(dt, RobotController.getBatteryVoltage());
        });

        simNotifier.startPeriodic(kSimLoopPeriod);
    }
}