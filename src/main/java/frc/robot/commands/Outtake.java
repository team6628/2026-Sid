package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class Outtake extends Command {

    private final Shooter shooter;
    private final Timer spinupTimer = new Timer();
    private final Timer jamTimer = new Timer();

    private double targetRPM;
    private boolean feeding = false;
    private boolean clearingJam = false;

    public Outtake(Shooter shooterSubsystem) {
        shooter = shooterSubsystem;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        spinupTimer.reset();
        spinupTimer.start();

        jamTimer.reset();
        clearingJam = false;
        feeding = false;
    }

    @Override
    public void execute() {
        // ---------------- Calculate target RPM ----------------
        double distance = shooter.getDistanceFeet();
        targetRPM = shooter.getTargetRPM(distance);
        shooter.setShooterRPM(targetRPM);

        // ---------------- Wait for spinup ----------------
        if (!feeding) {
            if (shooter.isStableAtSetpoint(targetRPM, 0.15, spinupTimer)) {
                feeding = true;
            } else return; // wait for spinup
        }

        // ---------------- Jam clearing ----------------
        if (clearingJam) {
            if (jamTimer.hasElapsed(0.2)) {
                clearingJam = false;
                jamTimer.stop();
            } else {
                shooter.reverseFeed();
                return;
            }
        }

        // ---------------- Feed ----------------
        shooter.feed();

        // ---------------- Detect jams ----------------
        if (shooter.isJammed(targetRPM)) {
            clearingJam = true;
            jamTimer.reset();
            jamTimer.start();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}