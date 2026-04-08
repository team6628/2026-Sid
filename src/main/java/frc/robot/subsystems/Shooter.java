package frc.robot.subsystems;

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
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;

public class Shooter extends SubsystemBase {

    // ======================== Motors ========================
    public final SparkMax motorA;
    public final SparkMax motorB;
    public final SparkClosedLoopController controllerA;
    public final SparkClosedLoopController controllerB;
    private final RelativeEncoder encoderA;
    private final RelativeEncoder encoderB;

    public final TalonFX motorC = new TalonFX(11); // Kraken

    // ======================== Constants ========================
    private static final double RPM_TOLERANCE = 100;
    private static final double RPM_DROP_THRESHOLD = 600;
    private static final double FEED_RPM = 3000;

    private final InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();

    // ======================== Limelight ========================
    private final NetworkTable limelightTable;
    private final NetworkTableEntry txEntry;
    private final NetworkTableEntry tyEntry;
    private final NetworkTableEntry taEntry;
    private final NetworkTableEntry tlEntry;
    private final NetworkTableEntry tidEntry;


    //Acceleration limiter
    SlewRateLimiter motorALimiter = new SlewRateLimiter(2000);
    SlewRateLimiter motorBLimiter = new SlewRateLimiter(2000);
    SlewRateLimiter motorCLimiter = new SlewRateLimiter(2000);


    public Shooter() {
        // REV Motors
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

        // Kraken TalonFX
        TalonFXConfiguration fxConfig = new TalonFXConfiguration();
        fxConfig.MotorOutput.Inverted = com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive;
        fxConfig.Slot0.kP = 0.1;
        fxConfig.Slot0.kI = 0.0;
        fxConfig.Slot0.kD = 0.0;
        fxConfig.Slot0.kV = 0.12;
        motorC.getConfigurator().apply(fxConfig);

        // Limelight
        limelightTable = NetworkTableInstance.getDefault().getTable("Limelight_6628");
        txEntry = limelightTable.getEntry("tx");
        tyEntry = limelightTable.getEntry("ty");
        taEntry = limelightTable.getEntry("ta");
        tlEntry = limelightTable.getEntry("tl");
        tidEntry = limelightTable.getEntry("tid");

        // RPM map
        rpmMap.put(4.0, 4500.0);
        rpmMap.put(6.0, 4000.0);
        rpmMap.put(8.0, 3650.0);
    }

    // ======================== Motor Control ========================
    public void setShooterRPM(double rpm) {

        double filteredA = motorALimiter.calculate(-rpm);
        double filteredB = motorBLimiter.calculate(rpm);
        double filteredC = motorCLimiter.calculate(rpm);

        controllerA.setReference(-filteredA, ControlType.kVelocity);
        controllerB.setReference(filteredB, ControlType.kVelocity);
        motorC.setControl(new VelocityDutyCycle(filteredC / 60.0));
    }

    public void stop() {
        motorA.stopMotor();
        motorB.stopMotor();
        motorC.setControl(new DutyCycleOut(0));
    }

    // ======================== Feed Control ========================
    public void feed() {
        controllerA.setReference(-FEED_RPM, ControlType.kVelocity);
        controllerB.setReference(FEED_RPM, ControlType.kVelocity);
    }

    public void reverseFeed() {
        controllerA.setReference(FEED_RPM, ControlType.kVelocity);
        controllerB.setReference(-FEED_RPM, ControlType.kVelocity);
    }

    // ======================== Feed / Intake ========================
    public void intake() {
        controllerA.setReference(motorALimiter.calculate(-4000), ControlType.kVelocity);
        controllerB.setReference(motorBLimiter.calculate(-4000), ControlType.kVelocity);
    }

    public void dump() {
        controllerA.setReference(motorALimiter.calculate(4000), ControlType.kVelocity);
        controllerB.setReference(motorBLimiter.calculate(-4000), ControlType.kVelocity);
    }

    // ======================== Sensors / State ========================
    public double getMotorARPM() { return encoderA.getVelocity(); }
    public double getMotorBRPM() { return encoderB.getVelocity(); }

    public boolean atSetpoint(double targetRPM) {
        return Math.abs(getMotorARPM() - targetRPM) < RPM_TOLERANCE;
    }

    public boolean isStableAtSetpoint(double targetRPM, double time, Timer timer) {
        if (atSetpoint(targetRPM)) return timer.hasElapsed(time);
        timer.reset();
        timer.start();
        return false;
    }

    public boolean isJammed(double targetRPM) {
        return getMotorARPM() < (targetRPM - RPM_DROP_THRESHOLD) ||
                getMotorBRPM() < (targetRPM - RPM_DROP_THRESHOLD);
    }

    // ======================== Limelight Helpers ========================
    public boolean hasTarget() { return tidEntry.getDouble(0.0) > 0; }
    public double getTX() { return txEntry.getDouble(0.0); }
    public double getTY() { return tyEntry.getDouble(0.0); }
    public double getTA() { return taEntry.getDouble(0.0); }
    public double getTL() { return tlEntry.getDouble(0.0); }

    // Calculate distance in feet from TY or fallback to TA
    public double getDistanceFeet() {
        if (!hasTarget()) return 6.0; // fallback

        double ty = getTY();
        if (ty != 0) {
            double cameraAngleDeg = 25.0;
            double cameraHeightFeet = 0.5833;
            double targetHeightFeet = 6.5;
            double angleRad = Math.toRadians(cameraAngleDeg + ty);
            return (targetHeightFeet - cameraHeightFeet) / Math.tan(angleRad);
        }

        // Fallback using area
        double ta = getTA();
        if (ta <= 0) return 6.0;
        return Math.sqrt(1.0 / ta) * 2.0; // tune experimentally
    }

    public double getTargetRPM(double distanceFeet) {
        double baseRPM = rpmMap.get(distanceFeet);
        double voltage = RobotController.getBatteryVoltage();
        return baseRPM * (12.0 / voltage);
    }

    @Override
    public void periodic() {
        double distance = getDistanceFeet();
        double targetRPM = getTargetRPM(distance);

        SmartDashboard.putNumber("Motor A RPM", getMotorARPM());
        SmartDashboard.putNumber("Motor B RPM", getMotorBRPM());
        SmartDashboard.putNumber("Target RPM", targetRPM);
        SmartDashboard.putNumber("Shot Distance", distance);
        SmartDashboard.putNumber("Limelight TX", getTX());
        SmartDashboard.putNumber("Limelight TY", getTY());
        SmartDashboard.putNumber("Limelight TL", getTL());
    }
}