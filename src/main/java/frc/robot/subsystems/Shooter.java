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

public class Shooter extends SubsystemBase {

    private final SparkMax motorA;
    private final SparkMax motorB;

    private final RelativeEncoder encoderA;
    private final RelativeEncoder encoderB;

    private final SparkClosedLoopController controllerA;
    private final SparkClosedLoopController controllerB;

    private static final double SHOOT_RPM = 4000;

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
        controllerA.setReference(SHOOT_RPM, ControlType.kVelocity);
        controllerB.setReference(-SHOOT_RPM, ControlType.kVelocity);
    }

    @SuppressWarnings("removal")
    public void outtake() {
        controllerA.setReference(SHOOT_RPM, ControlType.kVelocity);
        controllerB.setReference(SHOOT_RPM, ControlType.kVelocity);
    }

    @SuppressWarnings("removal")
    public void dump() {
        controllerA.setReference(-SHOOT_RPM, ControlType.kVelocity);
        controllerB.setReference(SHOOT_RPM, ControlType.kVelocity);
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
}