package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final CommandJoystick joystick = new CommandJoystick(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Shooter shooter = new Shooter();
    private final Outtake outtake = new Outtake(shooter);
    private final Intake intake = new Intake(shooter);
    private SendableChooser<Command> autoChooser;

    public RobotContainer() {
        configureBindings();
        configureAutos();
    }

    private void configureBindings() {
        // Default driving
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() ->
                        drive.withVelocityX(-joystick.getY() * MaxSpeed)
                                .withVelocityY(-joystick.getX() * MaxSpeed)
                                .withRotationalRate(-joystick.getZ() * MaxAngularRate)
                )
        );

        // Idle when disabled
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // SysId routines
        joystick.button(7).and(joystick.button(4))
                .whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.button(7).and(joystick.button(3))
                .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.button(8).and(joystick.button(4))
                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.button(8).and(joystick.button(3))
                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Shooter controls
        joystick.button(1).whileTrue(new Outtake(shooter));
        joystick.button(2).whileTrue(new Intake(shooter));
    }

    private void configureAutos() {
        // ----- REGISTER GLOBAL EVENT COMMANDS -----
        NamedCommands.registerCommand("Intake", (intake));
        NamedCommands.registerCommand("Outtake", (outtake));

        // Load RobotConfig from GUI (or you can hardcode)
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();

            // Configure AutoBuilder for swerve drivetrain
            AutoBuilder.configure(
                    // Robot pose supplier
                    () -> drivetrain.getState().Pose,
                    // Method to reset odometry
                    drivetrain::resetPose,
                    // ChassisSpeeds supplier (robot relative)
                    () -> drivetrain.getKinematics().toChassisSpeeds(drivetrain.getState().ModuleStates),
                    // Method to apply chassis speeds
                    (speeds, feedforwards) -> drivetrain.setControl(
                            drive.withVelocityX(speeds.vxMetersPerSecond)
                                    .withVelocityY(speeds.vyMetersPerSecond)
                                    .withRotationalRate(speeds.omegaRadiansPerSecond)
                    ),
                    // Holonomic drive controller
                    new PPHolonomicDriveController(
                            new PIDConstants(5.0, 0.0, 0.0), // Translation
                            new PIDConstants(5.0, 0.0, 0.0)  // Rotation
                    ),
                    config,
                    // Red alliance mirroring
                    () -> {
                        var alliance = DriverStation.getAlliance();
                        if (alliance.isPresent()) {
                            return alliance.get() == DriverStation.Alliance.Red;
                        }
                        return false;
                    },
                    drivetrain
            );

        } catch (Exception e) {
            e.printStackTrace();
        }

        // Build auto chooser using PathPlanner-generated paths
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    // Return the selected autonomous command
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}