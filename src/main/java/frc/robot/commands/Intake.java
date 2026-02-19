package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class Intake extends Command {

    private final Shooter shooter;

    public Intake(Shooter subsystem) {
        shooter = subsystem;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.intake();
    }

    @Override
    public void execute() {
        // Nothing needed here unless you want variable speed
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until button released
    }
}
