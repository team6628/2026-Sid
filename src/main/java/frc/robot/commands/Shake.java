package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class Shake extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final Timer timer = new Timer();
    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();

    private boolean direction = true;

    // Adjustable parameters for shake strength
    private static final double TRANS_X_SPEED = 0.5;  // m/s, forward/backward
    private static final double TRANS_Y_SPEED = 0.5;  // m/s, left/right
    private static final double SWITCH_TIME = 0.12;   // seconds between flips

    public Shake(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        direction = true;
        timer.restart();
    }

    @Override
    public void execute() {
        // Flip signs based on direction for back-and-forth shaking
        double xVel = direction ? TRANS_X_SPEED : -TRANS_X_SPEED;
        double yVel = direction ? TRANS_Y_SPEED : -TRANS_Y_SPEED;

        drivetrain.setControl(
            drive.withVelocityX(xVel)
                 .withVelocityY(yVel)
        );

        if (timer.hasElapsed(SWITCH_TIME)) {
            direction = !direction; // flip direction
            timer.restart();
        }
    }

    @Override
    public boolean isFinished() {
        return false; // runs while button is held
    }

    @Override
    public void end(boolean interrupted) {
        // stop all motion when command ends
        drivetrain.setControl(
            drive.withVelocityX(0)
                 .withVelocityY(0)
                 .withRotationalRate(0)
        );
    }
}