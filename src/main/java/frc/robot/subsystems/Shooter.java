package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.LimelightHelpers;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// ===================== RIO LOGGING =====================
import edu.wpi.first.wpilibj.DataLogManager;

public class Shooter extends SubsystemBase {

    // ===================== MOTORS =====================
    public final SparkMax motorA;
    public final SparkMax motorB;
    public final SparkClosedLoopController controllerA;
    public final SparkClosedLoopController controllerB;
    private final RelativeEncoder encoderA;
    private final RelativeEncoder encoderB;

    public final TalonFX motorC = new TalonFX(11);

    // ===================== STATE =====================
    public enum ShooterState {
        IDLE,
        SHOOTING,
        FEEDING,
        INTAKING,
        DUMPING
    }

    private ShooterState state = ShooterState.IDLE;

    // ===================== CONSTANTS =====================
    private static final double RPM_TOLERANCE = 100;
    private static final double RPM_DROP_THRESHOLD = 600;

    private static final String LOG = "Shooter/";

    private final InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();

    // ===================== LIMITERS =====================
    private final SlewRateLimiter aLimiter = new SlewRateLimiter(2000);
    private final SlewRateLimiter bLimiter = new SlewRateLimiter(2000);

    // ===================== LIMELIGHT =====================
    private static final String LIMELIGHT = "limelight";

    // ===================== INIT =====================
    public Shooter() {

        motorA = new SparkMax(9, MotorType.kBrushless);
        motorB = new SparkMax(10, MotorType.kBrushless);

        SparkMaxConfig configA = new SparkMaxConfig();
        SparkMaxConfig configB = new SparkMaxConfig();

        configB.inverted(true);

        configA.closedLoop.p(0.0002).i(0).d(0).velocityFF(0.00018);
        configB.closedLoop.p(0.0002).i(0).d(0).velocityFF(0.00018);

        motorA.configure(configA, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motorB.configure(configB, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoderA = motorA.getEncoder();
        encoderB = motorB.getEncoder();

        controllerA = motorA.getClosedLoopController();
        controllerB = motorB.getClosedLoopController();

        TalonFXConfiguration fx = new TalonFXConfiguration();
        fx.Slot0.kP = 0.1;
        fx.Slot0.kV = 0.12;

        motorC.getConfigurator().apply(fx);

        // RPM curve
        rpmMap.put(4.0, 3400.0);
        rpmMap.put(6.0, 3750.0);
        rpmMap.put(8.0, 4250.0);
    }

    // ===================== CONTROL =====================

    public void setShooterRPM(double rpm) {

        state = ShooterState.SHOOTING;

        double batteryComp = 12.0 / RobotController.getBatteryVoltage();
        double target = rpm * batteryComp;

        controllerA.setReference(-target, ControlType.kVelocity);
        controllerB.setReference(target, ControlType.kVelocity);

        motorC.setControl(new VelocityDutyCycle(target / 60.0));

        log("CommandRPM", rpm);
        log("BatteryComp", batteryComp);
    }

    public void stop() {
        state = ShooterState.IDLE;

        motorA.stopMotor();
        motorB.stopMotor();
        motorC.setControl(new DutyCycleOut(0));

        log("Stopped", 1);
    }

    // ===================== MODES =====================

    public void feed() {
        state = ShooterState.FEEDING;

        controllerA.setReference(3000, ControlType.kVelocity);
        controllerB.setReference(3000, ControlType.kVelocity);

        log("Mode", "FEED");
    }

    public void intake() {
        state = ShooterState.INTAKING;

        controllerA.setReference(4000, ControlType.kVelocity);
        controllerB.setReference(4000, ControlType.kVelocity);

        log("Mode", "INTAKE");
    }

    public void dump() {
        state = ShooterState.DUMPING;

        controllerA.setReference(-4000, ControlType.kVelocity);
        controllerB.setReference(-4000, ControlType.kVelocity);

        log("Mode", "DUMP");
    }

    public boolean isStableAtSetpoint(double targetRPM, double toleranceSamples, Timer spinupTimer) {

        double error = Math.abs(getMotorARPM() - targetRPM);

        if (error < RPM_TOLERANCE) {
            return true; // already stable
        }

        return false;
    }

    // ===================== SENSORS =====================

    public double getMotorARPM() {
        return encoderA.getVelocity();
    }

    public double getMotorBRPM() {
        return encoderB.getVelocity();
    }

    public boolean atSetpoint(double target) {
        return Math.abs(getMotorARPM() - target) < RPM_TOLERANCE;
    }

    public boolean isJammed(double target) {
        return getMotorARPM() < (target - RPM_DROP_THRESHOLD);
    }

    // ===================== LIMELIGHT =====================

    public boolean hasTarget() {
        return LimelightHelpers.getTV(LIMELIGHT);
    }

    public double getTX() {
        return LimelightHelpers.getTX(LIMELIGHT);
    }

    public double getTY() {
        return LimelightHelpers.getTY(LIMELIGHT);
    }

    public LimelightHelpers.PoseEstimate getPose() {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue(LIMELIGHT);
    }

    // ===================== DISTANCE =====================

    public double getDistanceFeet() {

        var est = getPose();

        if (est == null || est.tagCount == 0) {
            log("Vision", "NO_TAG");
            return 6.0;
        }

        double dx = est.pose.getX();
        double dy = est.pose.getY();

        double dist = Math.hypot(dx, dy);

        log("Distance", dist);

        return dist;
    }

    public double getTargetRPM(double distanceFeet) {
        double base = rpmMap.get(distanceFeet);
        double comp = base * (12.0 / RobotController.getBatteryVoltage());

        log("TargetRPM", comp);

        return comp;
    }

    // ===================== RIO LOGGING =====================

    private void log(String key, Object value) {
        DataLogManager.log(LOG + key + "=" + value);
    }

    // ===================== PERIODIC =====================

    @Override
    public void periodic() {

        double distance = getDistanceFeet();
        double target = getTargetRPM(distance);

        double a = getMotorARPM();
        double b = getMotorBRPM();

        log("State", state.name());
        log("MotorA", a);
        log("MotorB", b);
        log("ErrorA", target - a);
        log("ErrorB", target - b);

        log("HasTarget", hasTarget());
        log("TX", getTX());
        log("TY", getTY());
    }
}