// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; 
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AimAndDriveCommand;
import frc.robot.commands.SubsystemCommands;
import frc.robot.subsystems.ControllerHandler;
import frc.robot.subsystems.DriveControllerAdapter;
import frc.robot.subsystems.LemonLime;
import frc.robot.subsystems.stock.SwerveSubsystem;
import frc.robot.subsystems.wcp.*;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import java.io.File;
import java.util.Optional;

import swervelib.SwerveInputStream;

public class RobotContainer{
	private final ControllerHandler      control      = new ControllerHandler();
	private final DriveControllerAdapter driveAdapter = new DriveControllerAdapter(control);

	private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve/falcon"));

	private final Feeder	mFeeder		 = new Feeder();
	private final Floor		mFloor		 = new Floor();
	private final Hanger	mHanger		 = new Hanger();
	private final Hood		mHood		 = new Hood();
	private final Intake	mIntake		 = new Intake();
	private final Limelight	mLimelight	 = new Limelight(Constants.Ports.FORWARD_LIMELIGHT);
	private final LemonLime mLemonLime   = new LemonLime(drivebase);
	private final Shooter	mShooter	 = new Shooter();

	// Establish a Sendable Chooser that will be able to be sent to the
	// SmartDashboard, allowing selection of desired auto
	private final SendableChooser<Command> autoChooser;

	// Converts driver input into a field-relative ChassisSpeeds that is controlled
	// by angular velocity.
	SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
																																() -> driveAdapter.getDriveY(),
																																() -> driveAdapter.getDriveX())
																														.withControllerRotationAxis(control::d_rightX)
																														.deadband(OperatorConstants.DEADBAND)
																														.scaleTranslation(0.8)
																														.allianceRelativeControl(true);

	/**
	 * Clone's the angular velocity input stream and converts it to a robotRelative
	 * input stream.
	 */
	SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
																	.allianceRelativeControl(false);

	SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
																			() -> -control.d_leftY(),
																			() -> -control.d_leftX())
																	.withControllerRotationAxis(() -> control.d_rightX())
																	.deadband(OperatorConstants.DEADBAND)
																	.scaleTranslation(0.8)
																	.allianceRelativeControl(true);

	// Derive the heading axis with math!
	SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
		.withControllerHeadingAxis(() -> Math.sin(control.d_getAxis(2) * Math.PI) * (Math.PI * 2),
															 () -> Math.cos(control.d_getAxis(2) * Math.PI) * (Math.PI * 2))
		.headingWhile(true)
		.translationHeadingOffset(true)
		.translationHeadingOffset(Rotation2d.fromDegrees(0));

	Command Auto_Aim_Start	= mLemonLime.getEnableCommand();
	Command Auto_Aim_Stop	= mLemonLime.getDisableCommand();
	Command Intake_Open		= mIntake.intakeCommand();
	Command Intake_Halt		= mIntake.haltCommand();
	Command Intake_Close	= mIntake.closeCommand();
	Command Intake_Pulse	= mIntake.agitateCommand();
	Command Calibrate		= mIntake.calibrateCommand().alongWith(mHanger.homingCommand());
	Command Climber_Up		= mHanger.positionCommand(Hanger.Position.HANGING);
	Command Climber_Down	= mHanger.positionCommand(Hanger.Position.HUNG);
	// Command Fire			= mShooter.spinUpCommand(5000).raceWith(Commands.waitSeconds(5)).andThen(mFloor.feedCommand().alongWith(mFeeder.feedCommand())); // Fire
	// Command Stop_Firing		= mShooter.spinUpCommand(0).alongWith(mFloor.idle()).alongWith(mFeeder.idle()); // Stop Firing
	// Command Feeder_Reverse  = mFloor.reverseCommand().alongWith(mFeeder.reverseCommand());
	// Command Stop_Firing		= mShooter.spinUpCommand(0).alongWith(mFloor.idle()).alongWith(mFeeder.idle()); // Stop Firing
	Command Spool_Up		= mShooter.spinUpCommand(5000).raceWith(Commands.waitSeconds(5));
	Command Spool_Down		= mShooter.spinUpCommand(0).raceWith(Commands.waitSeconds(10));
	Command Feeder_Reverse  = mFloor.reverseCommand().alongWith(mFeeder.reverseCommand());
	Command Feeder_Forward	= mFloor.feedCommand().alongWith(mFeeder.feedCommand());
	Command Feeder_Stop		= mFloor.idle().alongWith(mFeeder.idle());
	Command Launcher_Fire	=  (mFloor.reverseCommand()
								.alongWith(
									mFeeder.reverseCommand()
								).raceWith(
									mShooter.spinUpCommand(5000)
								)).raceWith(
									Commands.waitSeconds(5)
								).andThen(
									mFloor.feedCommand()
									.alongWith(
										mFeeder.feedCommand()
								));
	Command Launcher_Stop	= 	mFloor.reverseCommand()
								.andThen(
									mFeeder.reverseCommand()
								).raceWith(
									Commands.none()
								).andThen(
									mShooter.spinUpCommand(0)
								).andThen(
									Commands.waitSeconds(2)
								).andThen(
									mFloor.idle()
									.andThen(
										mFeeder.idle()
								));
	Command Launcher_Unjam  = 	mFloor.reverseCommand()
								.alongWith(
									mFeeder.reverseCommand()
								).alongWith(
									mShooter.spinUpCommand(5000)
									.raceWith( Commands.waitSeconds(.4) )
									.andThen( mShooter.spinUpCommand(0   ) )
									.raceWith( Commands.waitSeconds(.4) )
									.repeatedly()
								);
	
	Command ToggleVisionDriving = mLemonLime.toggleVisionDriving();

	SubsystemCommands subsystemCommands = new SubsystemCommands(drivebase, mIntake, mFloor, mFeeder, mShooter, mHood, mHanger);
	Command aimAndShoot = subsystemCommands.aimAndShoot();
	
	public RobotContainer() {
		// Configure the trigger bindings
		configureBindings();
		driveAdapter.setVehicleYawSupplier(()->drivebase.getHeading().getRadians());
		DriverStation.silenceJoystickConnectionWarning(true);
		drivebase.resetOdometry(new Pose2d(Inches.of(158.84), Inches.of(181.46-143),new Rotation2d(Degrees.of(90))));
		
		//Create the NamedCommands that will be used in PathPlanner
		NamedCommands.registerCommand("Placeholder Command", Commands.print(" <!> Placeholder Command Triggered"));
		NamedCommands.registerCommand("Placeholder Command II", Commands.print(" <!> Placeholder Command II Triggered"));
		NamedCommands.registerCommand("Start Auto-Aim",Auto_Aim_Start);
		NamedCommands.registerCommand("Stop Auto-Aim",Auto_Aim_Stop);
		NamedCommands.registerCommand("Open & Activate Intake",Intake_Open);
		NamedCommands.registerCommand("Open & Disable Intake",Intake_Halt);
		NamedCommands.registerCommand("Close & Disable Intake",Intake_Close);
		NamedCommands.registerCommand("Pulse Intake (as adjetator)",Intake_Pulse);
		NamedCommands.registerCommand("Calibrate Intake",Calibrate);
		NamedCommands.registerCommand("Extend Climber",Climber_Up);
		NamedCommands.registerCommand("Retract Climber",Climber_Down);
		// NamedCommands.registerCommand("Begin Firing",Fire);
		// NamedCommands.registerCommand("Stop Firing",Stop_Firing);


		//Have the autoChooser pull in all PathPlanner autos as options
		autoChooser = AutoBuilder.buildAutoChooser();
		autoChooser.setDefaultOption("Do Nothing", Commands.none());
		autoChooser.addOption("Drive Forward", drivebase.driveForward().withTimeout(1));
		
		
		//Put the autoChooser on the SmartDashboard
		SmartDashboard.putData("Auto Chooser", autoChooser);
	}

	private void configureBindings() {
		//Pass vision data to the swerve drive system

		mLimelight.setDefaultCommand(updateVisionCommand());

		// Command driveFieldOrientedAnglularVelocity    = drivebase.driveFieldOriented(driveAngularVelocity);
		Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);

		if (RobotBase.isSimulation())
			drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
		else
			drivebase.setDefaultCommand(drivebase.driveCommand(driveAdapter::getDriveY, 
			driveAdapter::getDriveX, 
			// ()->(driveAdapter.getDriveR()+mLemonLime.getVisualJoyStick()), false));//
			()->(driveAdapter.getDriveR()), false));//

//  drivebase.driveToPose(new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0));

		control.d_R1().onTrue 							(Commands.runOnce(()->driveAdapter.setFieldOriented(true)));
		control.d_R1().onFalse							(Commands.runOnce(()->driveAdapter.setFieldOriented(false)));
		control.d_LSB().and(control.d_RSB()).onTrue		(Commands.runOnce(drivebase::lock).repeatedly());
		control.d_square().onTrue						(Commands.runOnce(drivebase::zeroGyro));

		control.d_triangle().whileTrue(aimAndShoot);

		// control.d_triangle().onTrue						(Auto_Aim_Start); // Auto-Aim
		// control.d_triangle().onFalse					(Auto_Aim_Stop ); // Auto-Aim
		
		control.h_povUp().onTrue						(Intake_Open ); // Open & Activate Intake
		control.h_povLeft().onTrue						(Intake_Halt ); // Open & Disable Intake
		control.h_povDown().onTrue						(Intake_Close); // Close & Disable Intake
		control.h_povRight().onTrue						(Intake_Pulse); // Pulse Intake (as adjetator)
		control.h_menu().onTrue							(Calibrate); // zero the intake via the limit switch
		control.h_L2().onTrue							(Climber_Up  .andThen(Commands.print(">> Up"))); // Extend Climber
		control.h_L2().onFalse							(Climber_Down.andThen(Commands.print(">> Down"))); // Retract Climber

		control.h_R2().onTrue							(Launcher_Fire ); // Fire
		control.h_R2().or(control.h_cross()).onFalse	(Launcher_Stop ); // Stop Firing
		control.h_cross().onTrue						(Launcher_Unjam); // Backfeed
	} 

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// Pass in the selected auto from the SmartDashboard as our desired autnomous
		// commmand
		return autoChooser.getSelected();
	}

	public void setMotorBrake(boolean brake) {
		drivebase.setMotorBrake(brake);
	}

	private Command updateVisionCommand() {
        return mLimelight.run(() -> {
            final Pose2d currentRobotPose = drivebase.getPose();
            final Optional<Limelight.Measurement> measurement = mLimelight.getMeasurement(currentRobotPose);
            measurement.ifPresent(m -> {
                drivebase.getSwerveDrive().addVisionMeasurement(
                    m.poseEstimate.pose, 
                    m.poseEstimate.timestampSeconds,
                    m.standardDeviations
                );
            });
        })
        .ignoringDisable(true);
    }
}