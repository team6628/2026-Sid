package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

public class Align extends Command {

    private final CommandSwerveDrivetrain m_drivetrain;
    private final Shooter shooter;
    
    // Private request with 0 deadband. 
    // Switched to OpenLoopVoltage to avoid Licensing errors from your logs.
    private final SwerveRequest.RobotCentric visionRequest = new SwerveRequest.RobotCentric()
        .withDeadband(0)
        .withRotationalDeadband(0)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private static final double kP = -0.04;
    private static final double kStatic = 0.05; 
    private static final double MAX_SPEED = 0.6; // % power (0.0 to 1.0)
    private static final double TOLERANCE = 1.2; 
    private static final double SEARCH_SPEED = 0.3; // % power

    private int lostTargetCounter = 0;
    private final int WAIT_THRESHOLD = 25; // 500ms

    public Align(CommandSwerveDrivetrain drivetrain, Shooter shooterSubsystem) {
        m_drivetrain = drivetrain;
        shooter = shooterSubsystem;
        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize() {
        lostTargetCounter = 0;
        System.out.println("[Align] STARTING");
    }

    @Override
public void execute() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    double tv = table.getEntry("tv").getDouble(0.0);
    boolean hasTarget = (tv > 0);
    
    double tx = shooter.getCorrectedTX();
    double rotationOutput = 0;

    if (hasTarget) {
        lostTargetCounter = 0;
        if (Math.abs(tx) > TOLERANCE) {
            // Logic: (tx * kP) + Feedforward
            rotationOutput = (tx * kP) + (Math.signum(tx) * kStatic);
        }
    } else {
        lostTargetCounter++;
        if (lostTargetCounter >= WAIT_THRESHOLD) {
            rotationOutput = SEARCH_SPEED;
            System.out.println(">>> [STATE] SEARCHING (Speed: " + SEARCH_SPEED + ")");
        } else {
            rotationOutput = 0;
            System.out.println(">>> [STATE] WAITING FOR TARGET (" + lostTargetCounter + "/" + WAIT_THRESHOLD + ")");
        }
    }

    // Clamp Output
    rotationOutput = Math.max(-MAX_SPEED, Math.min(MAX_SPEED, rotationOutput));

    // --- DIAGNOSTIC LOGS ---
    System.out.print("V: " + (hasTarget ? "FOUND" : "LOST") + " | tx: " + String.format("%.2f", tx));
    

    System.out.print(" | Calc Out: " + String.format("%.3f", rotationOutput));

    // Apply the control
   m_drivetrain.setControl(
    visionRequest
        .withVelocityX(0)
        .withVelocityY(0)
        .withRotationalRate(rotationOutput * 12.0) // Apply the 12x multiplier here
);


    System.out.println(" | Req RotRate: " + String.format("%.3f", visionRequest.RotationalRate));
}

    @Override
    public boolean isFinished() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        double tv = table.getEntry("tv").getDouble(0.0);
        double tx = shooter.getCorrectedTX();
        return tv > 0 && Math.abs(tx) <= TOLERANCE;
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.setControl(visionRequest.withRotationalRate(0));
        System.out.println("[Align] ENDED. Interrupted: " + interrupted);
    }
}
