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
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ControllerHandler;
import frc.robot.subsystems.DriveControllerAdapter;
import frc.robot.subsystems.stock.SwerveSubsystem;

import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
	private final ControllerHandler      control      = new ControllerHandler();
	private final DriveControllerAdapter driveAdapter = new DriveControllerAdapter(control);

	private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve/falcon"));

	// Establish a Sendable Chooser that will be able to be sent to the SmartDashboard, allowing selection of desired auto
	private final SendableChooser<Command> autoChooser;

	// Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
	SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
																																() -> driveAdapter.getDriveY(),
																																() -> driveAdapter.getDriveX())
																														.withControllerRotationAxis(control::d_rightX)
																														.deadband(OperatorConstants.DEADBAND)
																														.scaleTranslation(0.8)
																														.allianceRelativeControl(true);

	/**
	 * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
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

	public RobotContainer() {
		// Configure the trigger bindings
		configureBindings();
		driveAdapter.setVehicleYawSupplier(()->drivebase.getHeading().getRadians());
		DriverStation.silenceJoystickConnectionWarning(true);
		
		//Create the NamedCommands that will be used in PathPlanner
		NamedCommands.registerCommand("Placeholder Command", Commands.print(" <!> Placeholder Command Triggered"));
		NamedCommands.registerCommand("Placeholder Command II", Commands.print(" <!> Placeholder Command II Triggered"));

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

		control.d_R1().onTrue 												(Commands.runOnce(()->driveAdapter.setFieldOriented(true)));
		control.d_R1().onFalse												(Commands.runOnce(()->driveAdapter.setFieldOriented(false)));
		control.d_LSB().and(control.d_RSB()).onTrue		(Commands.runOnce(drivebase::lock));
		control.d_square().onTrue											(Commands.runOnce(drivebase::zeroGyro));
		control.d_triangle().whileTrue								(Commands.run(()->{})); // TODO Auto-Aim
		
		control.h_povUp().onTrue											(Commands.run(()->{})); // TODO Open & Activate Intake
		control.h_povLeft().onTrue										(Commands.run(()->{})); // TODO Open & Disable Intake
		control.h_povDown().onTrue										(Commands.run(()->{})); // TODO Close & Disable Intake
		control.h_povRight().onTrue										(Commands.run(()->{})); // TODO Pulse Intake (as adjetator)
		control.h_L2().onTrue													(Commands.run(()->{})); // TODO Extend Climber
		control.h_L2().onFalse												(Commands.run(()->{})); // TODO Retract Climber

		control.h_R2().onTrue													(Commands.run(()->{})); // TODO Fire
		control.h_R2().onFalse												(Commands.run(()->{})); // TODO Stop Firing
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand()
	{
		// Pass in the selected auto from the SmartDashboard as our desired autnomous commmand 
		return autoChooser.getSelected();
	}

	public void setMotorBrake(boolean brake)
	{
		drivebase.setMotorBrake(brake);
	}
}