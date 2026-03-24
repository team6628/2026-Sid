package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class Outtake extends Command {

    private final Shooter shooter;
    private final Timer timer = new Timer();
    private boolean motorBStarted = false;

    public Outtake(Shooter subsystem) {
        shooter = subsystem;
        addRequirements(shooter);
    }

    @SuppressWarnings("removal")
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        motorBStarted = false;

        // Start motor A immediately
        shooter.controllerA.setReference(4500, com.revrobotics.spark.SparkBase.ControlType.kVelocity);
    }

    @SuppressWarnings("removal")
    @Override
    public void execute() {
        // Start motor B after 0.25 seconds
        if (!motorBStarted && timer.hasElapsed(1)) {
            shooter.controllerB.setReference(3000, com.revrobotics.spark.SparkBase.ControlType.kVelocity);
            motorBStarted = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return false; // runs while held
    }
}
