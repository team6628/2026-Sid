package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends SubsystemBase {

    public final SparkMax motorA;
    public final SparkMax motorB;

    private final RelativeEncoder encoderA;
    private final RelativeEncoder encoderB;

    public final SparkClosedLoopController controllerA;
    public final SparkClosedLoopController controllerB;

    private static final double INTAKE_RPM_A = 4000;
    private static final double INTAKE_RPM_B = 2000;

    private static final double SHOOT_RPM_A = 4500;
    private static final double SHOOT_RPM_B = 3000;

    @SuppressWarnings("removal")
    public Shooter() {

        motorA = new SparkMax(9, MotorType.kBrushless);
        motorB = new SparkMax(10, MotorType.kBrushless);

        SparkMaxConfig configA = new SparkMaxConfig();
        SparkMaxConfig configB = new SparkMaxConfig();

        configB.inverted(true);

        configA.closedLoop
                .p(0.0002)
                .i(0)
                .d(0)
                .velocityFF(0.00018);

        configB.closedLoop
                .p(0.0002)
                .i(0)
                .d(0)
                .velocityFF(0.00018);

        motorA.configure(configA, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motorB.configure(configB, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoderA = motorA.getEncoder();
        encoderB = motorB.getEncoder();

        controllerA = motorA.getClosedLoopController();
        controllerB = motorB.getClosedLoopController();
    }

    @SuppressWarnings("removal")
    public void intake() {
        controllerA.setReference(INTAKE_RPM_A, ControlType.kVelocity);
        controllerB.setReference(-INTAKE_RPM_B, ControlType.kVelocity);
    }

    @SuppressWarnings("removal")
    public void outtake() {
        controllerA.setReference(SHOOT_RPM_A, ControlType.kVelocity);
        controllerB.setReference(SHOOT_RPM_B, ControlType.kVelocity);
    }

    @SuppressWarnings("removal")
    public void dump() {
        controllerA.setReference(-INTAKE_RPM_A, ControlType.kVelocity);
        controllerB.setReference(INTAKE_RPM_B, ControlType.kVelocity);
    }

    public void stop() {
        motorA.stopMotor();
        motorB.stopMotor();
    }

    public double getMotorARPM() {
        return encoderA.getVelocity();
    }

    public double getMotorBRPM() {
        return encoderB.getVelocity();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Motor A RPM", getMotorARPM());
        SmartDashboard.putNumber("Shooter Motor B RPM", getMotorBRPM());
        SmartDashboard.putNumber("Shooter Target A", SHOOT_RPM_A);
        SmartDashboard.putNumber("Shooter Target B", SHOOT_RPM_B);
}
}