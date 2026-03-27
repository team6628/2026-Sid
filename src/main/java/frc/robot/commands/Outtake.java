package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
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
    }

    @Override
    public void execute() {
        // ---------------- Live Limelight 3 AprilTag data ----------------
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight"); // default pipeline
        double tid = table.getEntry("tid").getDouble(0.0);  // tag ID
        double ty  = table.getEntry("ty").getDouble(0.0);   // vertical offset
        double tl  = table.getEntry("tl").getDouble(0.0);   // horizontal offset / pipeline-specific

        boolean hasTarget = tid > 0;

        // Debug prints
        System.out.println("[Outtake] tid: " + tid + ", ty: " + ty + ", tl: " + tl);

        // ---------------- Compute distance to target ----------------
        double distanceFeet = hasTarget ? calculateDistance(ty) : 6.0; // fallback if no tag

        targetRPM = shooter.getTargetRPM(distanceFeet);
        shooter.setShooterRPM(targetRPM);

        // ---------------- Shooter spinup ----------------
        if (!feeding) {
            if (shooter.isStableAtSetpoint(targetRPM, 0.15, spinupTimer)) {
                feeding = true;
            } else {
                return; // wait until spinup is stable
            }
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

    // -------------------- Helper to calculate distance from vertical angle --------------------
    private double calculateDistance(double ty) {
        double limelightMountAngleDeg = 25.0;   // adjust to your actual mount angle
        double limelightHeightFeet = 0.5833;    // camera height
        double targetHeightFeet = 6.5;          // goal height

        double angleToGoalDeg = limelightMountAngleDeg + ty;
        double angleToGoalRad = Math.toRadians(angleToGoalDeg);

        double distance = (targetHeightFeet - limelightHeightFeet) / Math.tan(angleToGoalRad);
        System.out.println("[Outtake] Calculated distance: " + distance + " ft");
        return distance;
    }
}