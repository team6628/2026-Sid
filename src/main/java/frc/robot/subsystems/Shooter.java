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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Timer;

public class Shooter extends SubsystemBase {

    // REV Motors
    public final SparkMax motorA;
    public final SparkMax motorB;
    private final RelativeEncoder encoderA;
    private final RelativeEncoder encoderB;
    public final SparkClosedLoopController controllerA;
    public final SparkClosedLoopController controllerB;

    // Kraken
    public TalonFX motorC = new TalonFX(11);

    // Interpolation map
    private final InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();

    // Constants
    private static final double RPM_TOLERANCE = 100;
    private static final double RPM_DROP_THRESHOLD = 600;

    private static final double FEED_RPM = 3000;

    @SuppressWarnings("removal")
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

        // Kraken config
        TalonFXConfiguration fxConfig = new TalonFXConfiguration();
        fxConfig.MotorOutput.Inverted = com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive;
        fxConfig.Slot0.kP = 0.1;
        fxConfig.Slot0.kI = 0.0;
        fxConfig.Slot0.kD = 0.0;
        fxConfig.Slot0.kV = 0.12;
        motorC.getConfigurator().apply(fxConfig);

        initMaps();
    }

    private void initMaps() {
        rpmMap.put(4.0, 4500.0);
        rpmMap.put(6.0, 4000.0);
        rpmMap.put(8.0, 3650.0);
    }

    // ========================
    // RPM CONTROL
    // ========================
    public void setShooterRPM(double rpmA) {
        double rpmB = rpmA; // adjust ratio if needed
        double rpsC = rpmA / 60.0;

        controllerA.setReference(-rpmA, ControlType.kVelocity);
        controllerB.setReference(rpmB, ControlType.kVelocity);

        motorC.setControl(new VelocityDutyCycle(rpsC));
    }

    public void stop() {
        motorA.stopMotor();
        motorB.stopMotor();
        motorC.setControl(new DutyCycleOut(0));
    }

    // ========================
    // CALCULATION
    // ========================
    public double getTargetRPM(double distance) {
        double baseRPM = rpmMap.get(distance);
        double voltage = RobotController.getBatteryVoltage();
        return baseRPM * (12.0 / voltage);
    }

    // ========================
    // FEED SYSTEM
    // ========================
    public void feed() {
        controllerA.setReference(-FEED_RPM, ControlType.kVelocity);
        controllerB.setReference(FEED_RPM, ControlType.kVelocity);
    }

    public void reverseFeed() {
        controllerA.setReference(FEED_RPM, ControlType.kVelocity);
        controllerB.setReference(-FEED_RPM, ControlType.kVelocity);
    }

    public void intake() {
        controllerA.setReference(-4000, ControlType.kVelocity);
        controllerB.setReference(-4000, ControlType.kVelocity);
    }

    public void dump() {
        controllerA.setReference(4000, ControlType.kVelocity);
        controllerB.setReference(4000, ControlType.kVelocity);
    }

    // ========================
    // SENSORS
    // ========================
    public double getMotorARPM() {
        return encoderA.getVelocity();
    }

    public double getMotorBRPM() {
        return encoderB.getVelocity();
    }

    // ========================
    // LOGIC
    // ========================
    public boolean atSetpoint(double targetRPM) {
        return Math.abs(getMotorARPM() - targetRPM) < RPM_TOLERANCE;
    }

    public boolean isStableAtSetpoint(double targetRPM, double time, Timer timer) {
        if (atSetpoint(targetRPM)) {
            return timer.hasElapsed(time);
        } else {
            timer.reset();
            timer.start();
            return false;
        }
    }

    public boolean isJammed(double targetRPM) {
        return getMotorARPM() < (targetRPM - RPM_DROP_THRESHOLD)
            || getMotorBRPM() < (targetRPM - RPM_DROP_THRESHOLD);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Motor A RPM", getMotorARPM());
        SmartDashboard.putNumber("Motor B RPM", getMotorBRPM());
        SmartDashboard.putNumber("Shot Distance", 6.0); //TODO: change to limelight
    }
}