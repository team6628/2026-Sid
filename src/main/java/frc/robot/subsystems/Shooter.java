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

public class Shooter extends SubsystemBase {

    // REV Motors
    public final SparkMax motorA;
    public final SparkMax motorB;
    private final RelativeEncoder encoderA;
    private final RelativeEncoder encoderB;
    public final SparkClosedLoopController controllerA;
    public final SparkClosedLoopController controllerB;

    // Kraken / TalonFX
    private final TalonFX motorC = new TalonFX(11);

    // RPM targets
    private static final double INTAKE_RPM_A = 4000;
    private static final double INTAKE_RPM_B = 2000;
    private static final double INTAKE_RPM_C = 3000;
    private static final double SHOOT_RPM_A = 4500;
    private static final double SHOOT_RPM_B = 3000;
    private static final double SHOOT_RPM_C = 4500;

    @SuppressWarnings("removal")
    public Shooter() {

        // Spark MAX setup
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

        // TalonFX / Kraken setup
        TalonFXConfiguration fxConfig = new TalonFXConfiguration();
        fxConfig.Slot0.kP = 0.1;
        fxConfig.Slot0.kI = 0.0;
        fxConfig.Slot0.kD = 0.0;
        fxConfig.Slot0.kV = 0.12;
        motorC.getConfigurator().apply(fxConfig);

        // STOP TalonFX at startup
        motorC.setControl(new DutyCycleOut(0));

        // STOP Sparks at startup
        motorA.stopMotor();
        motorB.stopMotor();
    }

    // Intake
    public void intake() {
        controllerA.setReference(INTAKE_RPM_A, ControlType.kVelocity);
        controllerB.setReference(-INTAKE_RPM_B, ControlType.kVelocity);

        VelocityDutyCycle velocityRequest = new VelocityDutyCycle(INTAKE_RPM_C / 60.0);
        motorC.setControl(velocityRequest);
    }

    // Shoot
    public void outtake() {
        controllerA.setReference(SHOOT_RPM_A, ControlType.kVelocity);
        controllerB.setReference(SHOOT_RPM_B, ControlType.kVelocity);

        VelocityDutyCycle velocityRequest = new VelocityDutyCycle(SHOOT_RPM_C / 60.0);
        motorC.setControl(velocityRequest);
    }

    // Dump / reverse
    public void dump() {
        controllerA.setReference(-INTAKE_RPM_A, ControlType.kVelocity);
        controllerB.setReference(INTAKE_RPM_B, ControlType.kVelocity);

        VelocityDutyCycle velocityRequest = new VelocityDutyCycle(-INTAKE_RPM_C / 60.0);
        motorC.setControl(velocityRequest);
    }

    // Stop all motors
    public void stop() {
        motorA.stopMotor();
        motorB.stopMotor();
        motorC.setControl(new DutyCycleOut(0));
    }

    // Getters
    public double getMotorARPM() {
        return encoderA.getVelocity();
    }

    public double getMotorBRPM() {
        return encoderB.getVelocity();
    }

    public double getMotorCRPS() {
        return motorC.getVelocity().getValueAsDouble();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Motor A RPM", getMotorARPM());
        SmartDashboard.putNumber("Shooter Motor B RPM", getMotorBRPM());
        SmartDashboard.putNumber("Shooter Motor C RPS", getMotorCRPS());

        SmartDashboard.putNumber("Target A RPM", SHOOT_RPM_A);
        SmartDashboard.putNumber("Target B RPM", SHOOT_RPM_B);
        SmartDashboard.putNumber("Target C RPM", SHOOT_RPM_C);
    }
}