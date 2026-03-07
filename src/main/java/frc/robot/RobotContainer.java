// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.PrepareShotCommand;
import frc.robot.commands.SubsystemCommands;
import frc.robot.subsystems.AutoCompiler;
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
	Command Intake_Calibrate= mIntake.calibrateCommand();
	Command Climber_Down	= mHanger.positionCommand(Hanger.Position.DOWN);
	Command Climber_Up		= mHanger.positionCommand(Hanger.Position.UP);

	Command FH_Hopper_Aim   = PrepareShotCommand.aimTargetStaticCommand(mShooter,mHood,()->drivebase.getPose());
	Command FH_Downrange    = PrepareShotCommand.aimDownrangeCommand(mShooter,mHood,()->drivebase.getPose());
	Command FH_Stop			= PrepareShotCommand.haltCommand(mShooter,mHood,()->drivebase.getPose());
	Command Feeder_Reverse  = mFloor.reverseCommand().alongWith(mFeeder.reverseCommand());
	Command Feeder_Forward	= mFloor.feedCommand().alongWith(mFeeder.feedCommand());
	Command Feeder_Stop		= mFloor.idle().alongWith(mFeeder.idle());

	Command Launcher_Unjam  = 	mFloor.reverseCommand().alongWith(mFeeder.reverseCommand());
	
	SubsystemCommands subsystemCommands = new SubsystemCommands(
		drivebase,mIntake,mFloor,mFeeder,mShooter,mHood,mHanger,
		driveAdapter::getDriveY, driveAdapter::getDriveX);
	Command aimAndShoot = subsystemCommands.aimAndShoot();
	
	public RobotContainer() {
		// Configure the trigger bindings
		configureBindings();
		driveAdapter.setVehicleYawSupplier(()->drivebase.getHeading().getRadians());
		DriverStation.silenceJoystickConnectionWarning(true);
		drivebase.resetOdometry(new Pose2d(Inches.of(158.84), Inches.of(181.46-143),new Rotation2d(Degrees.of(90))));

		AutoCompiler.SetUpCompiler();
		driveAdapter.setFieldOriented(false);
		
		//Create the NamedCommands that will be used in PathPlanner
		NamedCommands.registerCommand("Placeholder Command", Commands.print(" <!> Placeholder Command Triggered"));
		NamedCommands.registerCommand("Placeholder Command II", Commands.print(" <!> Placeholder Command II Triggered"));
		NamedCommands.registerCommand("Do Nothing", Commands.none());
		NamedCommands.registerCommand("Start Auto-Azimuth",Auto_Aim_Start);
		NamedCommands.registerCommand("Stop Auto-Azimuth",Auto_Aim_Stop);

		NamedCommands.registerCommand("Open & Activate Intake",Intake_Open);
		NamedCommands.registerCommand("Open & Disable Intake",Intake_Halt);
		NamedCommands.registerCommand("Close & Disable Intake",Intake_Close);
		NamedCommands.registerCommand("Pulse Intake (as adjetator)",Intake_Pulse);
		NamedCommands.registerCommand("Calibrate Intake",Intake_Calibrate);

		NamedCommands.registerCommand("Spool Up for Hopper", FH_Hopper_Aim);
		NamedCommands.registerCommand("Spool Up for Passing", FH_Downrange);
		NamedCommands.registerCommand("Spool Down", FH_Stop);
		NamedCommands.registerCommand("Feeder Forward", Feeder_Forward);
		NamedCommands.registerCommand("Feeder Reverse", Feeder_Reverse);
		NamedCommands.registerCommand("Feeder Stop", Feeder_Stop);

		NamedCommands.registerCommand("Extend Climber",Climber_Up);
		NamedCommands.registerCommand("Retract Climber",Climber_Down);

	}

	private void configureBindings() {
		//Pass vision data to the swerve drive system
		mLimelight.setDefaultCommand(updateVisionCommand());

		/*
		 * ----------------------------------------------------------
		 * Swerve drive calibration and configuration.
		 * ---------------------------------------------------------- 
		 */
		drivebase.setDefaultCommand(
			drivebase.driveCommand(
				driveAdapter::getDriveY, 
				driveAdapter::getDriveX, 
				()->(driveAdapter.getDriveR() + mLemonLime.getVisualJoyStick()),
			false));//

		control.d_LSB().and(control.d_RSB()).onTrue		(Commands.runOnce(drivebase::lock).repeatedly());
		
		/**
		 * ----------------------------------------------------------
		 * Intake related code for feeding balls into shooter.
		 * ---------------------------------------------------------- 
		 */
		control.h_povUp().onTrue						(Intake_Open ); // Open & Activate Intake
		control.h_povLeft().onTrue						(Intake_Halt ); // Open & Disable Intake
		control.h_povDown().onTrue						(Intake_Close); // Close & Disable Intake
		control.h_povRight().onTrue						(Intake_Pulse); // Pulse Intake (as adjetator)
		control.h_menu().onTrue							(Intake_Calibrate); // zero the intake via the limit switch

		/**
		 * ----------------------------------------------------------
		 * Fire control.
		 * ----------------------------------------------------------
		 */
		control.h_R2().onTrue							(FH_Hopper_Aim ); // Fire
		control.h_R1().onTrue							(FH_Downrange ); // Fire
		control.h_R2().or(control.h_R1()).onFalse		(FH_Stop   ); // Stop Firing
		control.h_cross().onTrue						(Launcher_Unjam); // Backfeed
		control.h_circle().onTrue						(Feeder_Forward); // Backfeed
		control.h_circle().or(control.h_cross()).onFalse(Feeder_Stop); // Backfeed

		/**
		 * ----------------------------------------------------------
		 * Auto-aim functionality, both direction and distance.
		 * ----------------------------------------------------------
		 */
		control.d_triangle().onTrue(Auto_Aim_Start);
		control.d_triangle().onFalse(Auto_Aim_Stop);
	} 

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		AutoCompiler.loadAutonomousProgram(Constants.AutonConstants.AutonName, Constants.AutonConstants.FlipSide);
		return AutoCompiler.getAuto();//AutoCompiler.GetAutoBasic(Constants.AutonConstants.AutonName);
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