// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AutoAim;
import frc.robot.commands.IntakeGoToDefault;
import frc.robot.commands.autosCommands.AutoAprilLock;
import frc.robot.commands.autosCommands.AutoAutoAim;
import frc.robot.commands.teleop.AprilLock;
import frc.robot.commands.teleop.AprilLockShuttle;
import frc.robot.commands.teleop.GrabFuel;
import frc.robot.commands.teleop.TeleopSwerve;
import frc.robot.commands.teleop.ToggleIntakeReady;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.Kicker;
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
    private final XboxController driver_hid = driver.getHID();
    private final Joystick operator = new Joystick(Constants.Controller.OPERATOR_PORT);


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

    public final TargetPoses target = new TargetPoses();
    public final JoystickButton tmJoystickFaceButtonRight = new JoystickButton(operator , 5);
    public final JoystickButton tmJoystickFaceButtonLeft = new JoystickButton(operator , 4);
    public final JoystickButton tmJoystickTrigger = new JoystickButton(operator , 1);
    public final POVButton tmJoystickPovUp = new POVButton(operator, 0);
    public final POVButton tmJoystickPovDown = new POVButton(operator, 180);
    public final JoystickButton tmJoystickRightHandBottomLeft = new JoystickButton(operator , 9);
    public final JoystickButton tmJoystickRightHandBottomMiddle = new JoystickButton(operator , 10);
    public final JoystickButton tmJoystickRightHandBottomRight = new JoystickButton(operator , 11);
    public final JoystickButton tmJoystickRightHandTopLeft = new JoystickButton(operator , 8);
    public final JoystickButton tmJoystickRightHandTopMiddle = new JoystickButton(operator , 7);
    public final JoystickButton tmJoystickRightHandTopRight = new JoystickButton(operator , 6);
    public double shooterFudgeFactor;
    public double turretFudgeFactor;

    public RobotContainer() {
        NamedCommands.registerCommand("Print", new InstantCommand(() -> System.out.println("test")));

        NamedCommands.registerCommand("IntakeOut", new GrabFuel( intakePivot));
        NamedCommands.registerCommand("IntakeCollect", intake.spin(false));
        NamedCommands.registerCommand("IntakeReverseCollect", intake.spin(true));
        NamedCommands.registerCommand("Kicker", kicker.run(false));
        NamedCommands.registerCommand("Hopper", hopper.run(false));
        NamedCommands.registerCommand("Shoot", new AutoAutoAim(shooter, hood));
        NamedCommands.registerCommand("AutoLock" , new AutoAprilLock(turret));
        NamedCommands.registerCommand("StopIntake", intake.StopCommand());
        NamedCommands.registerCommand("StopKicker", kicker.stopCommand());
        NamedCommands.registerCommand("StopHopper", hopper.stop());
        NamedCommands.registerCommand("StopShooter", shooter.stopCommand());
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        
        
        

        // Warmup PathPlanner to avoid Java pauses
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());

        // booo I don't like this
        // thrustmaster controls

        this.shooterFudgeFactor = 0;
        this.turretFudgeFactor = 0;
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            new TeleopSwerve(
                drivetrain,
                () -> driver_hid.getLeftY(),
                () -> driver_hid.getLeftX(),
                () -> driver_hid.getRightX()
            )
        );

        // always try to go to default

        intakePivot.setDefaultCommand(new IntakeGoToDefault(intakePivot));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle)
            .ignoringDisable(true)
        );

        driver.a()
            .whileTrue(drivetrain.applyRequest(() -> brake));
        driver.b()
            .whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        // driver.leftBumper()
        //    .onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // intake ready
        driver.rightBumper()
            .whileTrue(new ToggleIntakeReady(intakePivot))
            .onFalse(new IntakeGoToDefault(intakePivot));

        // intake put up
        driver.rightTrigger()
            .whileTrue(new GrabFuel( intakePivot))
            .onFalse(new IntakeGoToDefault(intakePivot));

        driver.rightTrigger()
            .whileTrue(intake.spin(false))
            .onFalse(intake.StopCommand());

        // intake eject
        // driver.leftTrigger()
        //     .whileTrue(intake.spin(true))
        //     .onFalse(intake.StopCommand());

        drivetrain.registerTelemetry(logger::telemeterize);

        // operater

        // intake hopper 
        tmJoystickPovUp
            .whileTrue(hopper.run(false)
            .alongWith(intake.spin(false))
            .alongWith(kicker.setRPMCommand()))
            .onFalse(hopper.stop()
            .alongWith(kicker.stopCommand()));

        tmJoystickPovDown
            .whileTrue(hopper.run(true)
            .alongWith(intake.spin(true))
            .alongWith(kicker.run(true)))
            .onFalse(hopper.stop()
            .alongWith(kicker.stopCommand()));

        
        

        if (tmJoystickFaceButtonRight.getAsBoolean()) 
            target.setScoring();

        tmJoystickFaceButtonRight
            .toggleOnTrue(turret.aimAtPose(target.getTargetPose()));
        
        if (tmJoystickFaceButtonLeft.getAsBoolean()) 
            target.setShuttling();

        tmJoystickFaceButtonLeft
            .toggleOnTrue(turret.aimAtPose(target.getTargetPose())); 

        tmJoystickTrigger
            .whileTrue(new AutoAim(shooter, hood ,target.getTargetPose()))
            .onFalse(shooter.stopCommand()
            .alongWith(hood.stopCommand()));


        

        
        // Manual Turret movement code
        tmJoystickRightHandBottomLeft
            .whileTrue(new TurretRotate(turret, -0.025));
        tmJoystickRightHandBottomMiddle
            .whileTrue(new TurretRotate(turret, 0.025));

        // Manual Hood movement code
        tmJoystickRightHandTopLeft
            .whileTrue(hood.setPowerCommand(false));
        tmJoystickRightHandTopMiddle
            .whileTrue(hood.setPowerCommand(true));

        // Shooter speed manual change
        tmJoystickRightHandTopRight
            .onTrue(shooter.offsetIncrement());
        tmJoystickRightHandBottomRight
            .onTrue(shooter.offsetDecrement());

        
        

        // this needs to be refactors to the inline standerds

        // Hub shot

        // set to shuttling

        //fudge it


        // target.fudgeShooterFactor(drivetrain.getState().Pose ,operator.getY() * shooterFudgeFactor);

        // target.fudgeTurretFactor(operator.getTwist()* turretFudgeFactor);
       // shooterFudgeFactor = Constants.TargetPosesConstants.SHOOTER_SLIDER_VALUE * operator.getThrottle()+.01 * Constants.TargetPosesConstants.SHOOTER_BASE_VALUE;
        //turretFudgeFactor = Constants.TargetPosesConstants.TURRET_SLIDER_VALUE * operator.getThrottle()+.01 * Constants.TargetPosesConstants.TURRET_BASE_VALUE;
        field.getObject("target").setPoses(target.CurrentTarget);

        SmartDashboard.putBoolean("thustmaster" ,driver.a().getAsBoolean());
        

    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

}
