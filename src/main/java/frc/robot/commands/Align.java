package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Align extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final Shooter shooter;

    private final SwerveRequest.RobotCentric request =
            new SwerveRequest.RobotCentric()
                    .withDeadband(0)
                    .withRotationalDeadband(0)
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // ===================== TARGET CONFIG =====================
    // Set this to your HUB position on the field
    // (YOU MUST ADJUST THIS FOR YOUR FIELD)
    private static final Pose2d HUB_POSE = new Pose2d(
            8.0,   // X meters (example)
            4.0,   // Y meters (example)
            new Rotation2d(0)
    );

    // ===================== CONTROL CONSTANTS =====================
    private static final double kP = 3.0;
    private static final double MAX_ROT_SPEED = 2.5; // rad/s clamp

    private static final String LIMELIGHT = "limelight";

    public Align(CommandSwerveDrivetrain drivetrain, Shooter shooter) {
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        System.out.println("[ALIGN] Starting botPose alignment");
    }

    @Override
    public void execute() {

        // ===================== GET ROBOT POSE =====================
        Pose2d robotPose =
                LimelightHelpers.getBotPoseEstimate_wpiBlue(LIMELIGHT).pose;

        boolean valid = LimelightHelpers.getTV(LIMELIGHT);

        if (!valid || robotPose == null) {
            System.out.println("[ALIGN] No vision target - holding");
            drivetrain.setControl(
                    request.withVelocityX(0)
                            .withVelocityY(0)
                            .withRotationalRate(0)
            );
            return;
        }

        // ===================== VECTOR TO HUB =====================
        Translation2d robotPos = robotPose.getTranslation();
        Translation2d hubPos = HUB_POSE.getTranslation();

        Translation2d delta = hubPos.minus(robotPos);

        // Desired angle to hub
        double desiredAngle =
                Math.atan2(delta.getY(), delta.getX());

        double currentAngle =
                robotPose.getRotation().getRadians();

        // Angular error
        double error = normalizeAngle(desiredAngle - currentAngle);

        // Control output
        double output = kP * error;

        // Clamp
        output = Math.max(-MAX_ROT_SPEED, Math.min(MAX_ROT_SPEED, output));

        // ===================== APPLY CONTROL =====================
        drivetrain.setControl(
                request.withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(output)
        );

        // ===================== DEBUG =====================
        System.out.println(
                "[ALIGN] err(rad)=" + error +
                        " out=" + output +
                        " robot=(" + robotPos.getX() + "," + robotPos.getY() + ")"
        );
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(
                request.withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(0)
        );

        System.out.println("[ALIGN] Ended interrupted=" + interrupted);
    }

    @Override
    public boolean isFinished() {

        Pose2d robotPose =
                LimelightHelpers.getBotPoseEstimate_wpiBlue(LIMELIGHT).pose;

        if (robotPose == null) return false;

        Translation2d robotPos = robotPose.getTranslation();
        Translation2d hubPos = HUB_POSE.getTranslation();

        Translation2d delta = hubPos.minus(robotPos);

        double desiredAngle =
                Math.atan2(delta.getY(), delta.getX());

        double currentAngle =
                robotPose.getRotation().getRadians();

        double error = normalizeAngle(desiredAngle - currentAngle);

        return Math.abs(error) < 0.05; // ~3 degrees
    }

    // ===================== UTIL =====================

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}