package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.revrobotics.spark.SparkBase.ControlType;

public class Outtake extends Command {

    private final Shooter shooter;
    private final Timer timer = new Timer();
    
    private boolean feedersStarted = false;

    public Outtake(Shooter subsystem) {
        shooter = subsystem;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        feedersStarted = false;

       
        VelocityDutyCycle flywheelVelocity = new VelocityDutyCycle(75.0);
        shooter.motorC.setControl(flywheelVelocity);
    }

    @Override
    public void execute() {
        if (!feedersStarted && timer.hasElapsed(0.2)) {
            // Using the RPM constants from your Shooter subsystem logic
            shooter.controllerA.setSetpoint(-3000, ControlType.kVelocity);
            shooter.controllerB.setSetpoint(3000, ControlType.kVelocity);
            feedersStarted = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Always shut everything down
        shooter.stop();
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return false; // Run while button is held
    }
}