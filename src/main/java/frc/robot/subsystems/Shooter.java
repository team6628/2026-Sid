package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    private final SparkMax motorA;
    private final SparkMax motorB;

    private static final double SHOOT_SPEED = 0.8;

    public Shooter() {
        motorA = new SparkMax(9, SparkLowLevel.MotorType.kBrushless);
        motorB = new SparkMax(10, SparkLowLevel.MotorType.kBrushed);

        // Invert one motor so they spin opposite directions
        motorB.setInverted(true);
    }

    public void intake() {
        motorA.set(SHOOT_SPEED);
        motorB.set(SHOOT_SPEED);
    }

    public void outtake() {
        motorA.set(SHOOT_SPEED);
        motorB.set(-SHOOT_SPEED);
    }

    public void stop() {
        motorA.stopMotor();
        motorB.stopMotor();
    }
}
