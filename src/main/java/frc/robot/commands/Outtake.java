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

    public Outtake(Shooter subsystem) {
        shooter = subsystem;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        spinupTimer.reset();
        spinupTimer.start();

        jamTimer.stop();
        jamTimer.reset();

        

        feeding = false;
        clearingJam = false;

        double distance = edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
        .getNumber("Shot Distance", 6.0); // replace with Limelight
        targetRPM = shooter.getTargetRPM(distance);

        shooter.setShooterRPM(targetRPM);
    }

    @Override
    public void execute() {

        // Wait for stable RPM
        if (!feeding) {
            if (shooter.isStableAtSetpoint(targetRPM, 0.15, spinupTimer)) {
                feeding = true;
            } else {
                return;
            }
        }

        // Jam clearing
        if (clearingJam) {
            if (jamTimer.hasElapsed(0.2)) {
                clearingJam = false;
                jamTimer.stop();
            } else {
                shooter.reverseFeed();
                return;
            }
        }

        // Feed
        shooter.feed();

        // Detect jam
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

    private double getDistanceFeet() {
        return 6.0; // TODO: replace with Limelight
    }
}