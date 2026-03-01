// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ControllerHandler;
import frc.robot.subsystems.DriveControllerAdapter;
import frc.robot.subsystems.LemonLime;
import frc.robot.subsystems.stock.SwerveSubsystem;
import frc.robot.subsystems.wcp.*;

import java.io.File;
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
	private final Limelight	mLimelight	 = new Limelight("limelight");
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

	Command Auto_Aim_Start	= new InstantCommand(()->{}); // TODO Auto-Aim
	Command Auto_Aim_Stop	= new InstantCommand(()->{}); // TODO Auto-Aim
	Command Intake_Open		= mIntake.intakeCommand();
	Command Intake_Halt		= mIntake.haltCommand();
	Command Intake_Close	= mIntake.closeCommand();
	Command Intake_Pulse	= mIntake.agitateCommand();
	Command Intake_Calibrate= mIntake.calibrateCommand();
	Command Climber_Up		= mHanger.positionCommand(Hanger.Position.HANGING);
	Command Climber_Down	= mHanger.homingCommand();
	Command Fire			= mShooter.spinUpCommand(3500).andThen(mFloor.feedCommand().alongWith(mFeeder.feedCommand())); // Fire
	Command Stop_Firing		= mShooter.spinUpCommand(0).alongWith(mFloor.idle()).alongWith(mFeeder.idle()); // Stop Firing
	Command ToggleVisionDriving = mLemonLime.toggleVisionDriving();
	
	public RobotContainer() {
		// Configure the trigger bindings
		configureBindings();
		driveAdapter.setVehicleYawSupplier(()->drivebase.getHeading().getRadians());
		DriverStation.silenceJoystickConnectionWarning(true);
		
		//Create the NamedCommands that will be used in PathPlanner
		NamedCommands.registerCommand("Placeholder Command", Commands.print(" <!> Placeholder Command Triggered"));
		NamedCommands.registerCommand("Placeholder Command II", Commands.print(" <!> Placeholder Command II Triggered"));
		NamedCommands.registerCommand("Start Auto-Aim",Auto_Aim_Start);
		NamedCommands.registerCommand("Stop Auto-Aim",Auto_Aim_Stop);
		NamedCommands.registerCommand("Open & Activate Intake",Intake_Open);
		NamedCommands.registerCommand("Open & Disable Intake",Intake_Halt);
		NamedCommands.registerCommand("Close & Disable Intake",Intake_Close);
		NamedCommands.registerCommand("Pulse Intake (as adjetator)",Intake_Pulse);
		NamedCommands.registerCommand("Calibrate Intake",Intake_Calibrate);
		NamedCommands.registerCommand("Extend Climber",Climber_Up);
		NamedCommands.registerCommand("Retract Climber",Climber_Down);
		NamedCommands.registerCommand("Begin Firing",Fire);
		NamedCommands.registerCommand("Stop Firing",Stop_Firing);


		//Have the autoChooser pull in all PathPlanner autos as options
		autoChooser = AutoBuilder.buildAutoChooser();
		autoChooser.setDefaultOption("Do Nothing", Commands.none());
		autoChooser.addOption("Drive Forward", drivebase.driveForward().withTimeout(1));
		
		
		//Put the autoChooser on the SmartDashboard
		SmartDashboard.putData("Auto Chooser", autoChooser);
	}

	private void configureBindings() {
		// Command driveFieldOrientedAnglularVelocity    = drivebase.driveFieldOriented(driveAngularVelocity);
		Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);

		if (RobotBase.isSimulation())
			drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
		else
			drivebase.setDefaultCommand(drivebase.driveCommand(driveAdapter::getDriveY, driveAdapter::getDriveX, driveAdapter::getDriveR, false));

		/* Old Command Bindings (Obsolete and commented out)
		if (Robot.isSimulation()) {
			Pose2d target = new Pose2d(new Translation2d(1, 4),
																 Rotation2d.fromDegrees(90));
			//drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
			driveDirectAngleKeyboard.driveToPose(() -> target,
																					 new ProfiledPIDController(5, 0, 0,
																																		 new Constraints(5, 2)),
																					 new ProfiledPIDController(5, 0, 0,
																																		 new Constraints(Units.degreesToRadians(360),
																																										 Units.degreesToRadians(180))
																					 ));
			control.d_button(15).onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d())))); // mute
			control.d_triangle().whileTrue(drivebase.sysIdDriveMotorCommand());
			control.d_circle().whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
																										 () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));
		}

//  drivebase.driveToPose(new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0));

		if (DriverStation.isTest())
		{
			drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

			control.d_square().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
			control.d_menu().onTrue((Commands.runOnce(drivebase::zeroGyro))); // menu
			control.d_blink().whileTrue(drivebase.centerModulesCommand()); // wiper fluid
			control.d_L2().onTrue(Commands.none());
			control.d_R2().onTrue(Commands.none());
		} else
		{
			control.d_cross().onTrue((Commands.runOnce(drivebase::zeroGyro)));
			control.d_square().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
			control.d_menu().whileTrue(Commands.none());
			control.d_blink().whileTrue(Commands.none());
			control.d_L2().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
			control.d_R2().onTrue(Commands.none());
		}
		*/

		control.d_R1().onTrue 							(Commands.runOnce(()->driveAdapter.setFieldOriented(true)));
		control.d_R1().onFalse							(Commands.runOnce(()->driveAdapter.setFieldOriented(false)));
		control.d_LSB().and(control.d_RSB()).onTrue		(Commands.runOnce(drivebase::lock));
		control.d_square().onTrue						(Commands.runOnce(drivebase::zeroGyro));
		control.d_triangle().onTrue						(Auto_Aim_Start); // Auto-Aim
		control.d_triangle().onFalse					(Auto_Aim_Stop ); // Auto-Aim
		
		control.h_povUp().onTrue						(Intake_Open ); // Open & Activate Intake
		control.h_povLeft().onTrue						(Intake_Halt ); // Open & Disable Intake
		control.h_povDown().onTrue						(Intake_Close); // Close & Disable Intake
		control.h_povRight().onTrue						(Intake_Pulse); // Pulse Intake (as adjetator)
		control.h_menu().onTrue							(Intake_Calibrate); // zero the intake via the limit switch
		control.h_L2().onTrue							(Climber_Up  ); // Extend Climber
		control.h_L2().onFalse							(Climber_Down); // Retract Climber

		control.h_R2().onTrue							(Fire        ); // Fire
		control.h_R2().onFalse							(Stop_Firing ); // Stop Firing
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
}