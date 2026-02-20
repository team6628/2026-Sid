package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.Intake;
import frc.robot.commands.Outtake;

public class RobotContainer {

    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    /* ROBOT CENTRIC DRIVE */
    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

   

    private final CommandJoystick joystick = new CommandJoystick(0);

    private final Shooter shooter = new Shooter();

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {

        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getY() * MaxSpeed)
                     .withVelocityY(-joystick.getX() * MaxSpeed)
                     .withRotationalRate(-joystick.getZ() * MaxAngularRate)
            )
        );

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // SysId
        joystick.button(7).and(joystick.button(4))
            .whileTrue(drivetrain.sysIdDynamic(Direction.kForward));

        joystick.button(7).and(joystick.button(3))
            .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));

        joystick.button(8).and(joystick.button(4))
            .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));

        joystick.button(8).and(joystick.button(3))
            .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        //Button Bindings
        joystick.button(1).whileTrue(new Outtake(shooter));
        joystick.button(2).whileTrue(new Intake(shooter));

    }

    public Command getAutonomousCommand() {

        final var idle = new SwerveRequest.Idle();

        return Commands.sequence(
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),

            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                     .withVelocityY(0)
                     .withRotationalRate(0)
            ).withTimeout(5.0),

            drivetrain.applyRequest(() -> idle)
        );
    }
}
