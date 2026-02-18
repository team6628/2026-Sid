package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class Shooter extends SubsystemBase {

    private final SparkMax motorA;
    private final SparkMax motorB;

    private static final double SHOOT_SPEED = 0.9;

    public Shooter() {
        motorA = new SparkMax(9, SparkLowLevel.MotorType.kBrushless); // PWM port 0
        motorB = new SparkMax(10, SparkLowLevel.MotorType.kBrushless); // PWM port 1

        // Invert one motor so they spin opposite directions
        motorB.setInverted(true);
    }

    public void intake() {
        motorA.set(SHOOT_SPEED);
        motorB.set(SHOOT_SPEED);
    }

    public void outtake() {
        motorA.set(-SHOOT_SPEED);
        motorB.set(-SHOOT_SPEED);
    }

    public void stop() {
        motorA.stopMotor();
        motorB.stopMotor();
    }
}
