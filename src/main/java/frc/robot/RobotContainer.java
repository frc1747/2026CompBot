// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AprilLock;
import frc.robot.commands.IntakeOut;
import frc.robot.commands.IntakeSpin;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.TurretRotate;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // Control
    private final CommandXboxController driver = new CommandXboxController(Constants.Controller.DRIVER_PORT);
    private final CommandXboxController operator = new CommandXboxController(Constants.Controller.OPERATOR_PORT);
    private final DoubleSupplier translationSup = () -> driver.getRawAxis(XboxController.Axis.kLeftY.value); // forward/backward on left stick
    private final DoubleSupplier strafeSup = () -> driver.getRawAxis(XboxController.Axis.kLeftX.value); // right/left on left stick
    private final DoubleSupplier rotationSup = () -> driver.getRawAxis(XboxController.Axis.kRightX.value); // right/left on right stick 

    public static final Kicker kicker = new Kicker();
    public static final Hood hood = new Hood();

    public static final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public static final Intake intake = new Intake();
    public static final IntakePivot intakePivot = new IntakePivot();
    public static final Shooter shooter = new Shooter();
    public static final Hopper hopper = new Hopper();
    public static final Turret turret = new Turret();

    public static final Field2d field = new Field2d();

    public RobotContainer() {
        NamedCommands.registerCommand("Print", new InstantCommand(() -> System.out.println("test")));

        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            new TeleopSwerve(drivetrain, translationSup, strafeSup, rotationSup)
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driver.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driver.povDown().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // intake commands
        // this is broken cause no encoder
        driver.rightTrigger().whileTrue(new IntakeOut(intakePivot, Constants.Intake.INTAKE_PIVOT_TICK).alongWith(new IntakeSpin(intake, Constants.Intake.POWER)));

        // much slower for the moment
        driver.rightBumper().whileTrue(new TurretRotate(turret, 0.025));
        driver.leftBumper().whileTrue(new TurretRotate(turret, -0.025));

        // this is on operator for now
        operator.leftBumper().whileTrue(new IntakeSpin(intake, Constants.Intake.POWER));

        operator.a().onTrue(kicker.run())
                    .onFalse(kicker.stop());

        operator.x().whileTrue(hood.setPowerCommand(true))  // down
                    .onFalse(hood.stopCommand());
        operator.y().whileTrue(hood.setPowerCommand(false))  // up
                    .onFalse(hood.stopCommand());

        // safe middle angle
        operator.rightBumper().whileTrue(hood.goToAngleCommand(10.0))
                              .onFalse(hood.stopCommand());

        operator.rightTrigger().whileTrue(shooter.setPowerCommand(0.2))
                               .onFalse(shooter.stopCommand());

        operator.leftTrigger().whileTrue(hopper.run())
                              .onFalse(hopper.stop());

        drivetrain.registerTelemetry(logger::telemeterize);

        operator.b().whileTrue(new AprilLock(turret));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
