// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ControllerHandler;
import frc.robot.subsystems.DriveControllerAdapter;
import frc.robot.subsystems.AutomaticRanger;
import frc.robot.subsystems.LemonLime;
import frc.robot.subsystems.stock.SwerveSubsystem;
import frc.robot.subsystems.wcp.*;

import static edu.wpi.first.units.Units.Meters;

import java.io.File;
import java.util.Optional;

public class RobotContainer{
	private final ControllerHandler      control      = new ControllerHandler();
	private final DriveControllerAdapter driveAdapter = new DriveControllerAdapter(control);

	private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve/falcon"));

	private final Feeder	mFeeder		 = new Feeder();
	private final Floor		mFloor		 = new Floor();
	private final Hood		mHood		 = new Hood();
	private final Intake	mIntake		 = new Intake();
	private final LemonLime mLemonLime   = new LemonLime(drivebase);
	private final Shooter	mShooter	 = new Shooter();

	private final LimelightPositioning	mLimelightPositioner = new LimelightPositioning(Constants.Ports.FORWARD_LIMELIGHT);
	private final AutomaticRanger   	mRanger				 = new AutomaticRanger(mShooter, mHood, ()->mLemonLime.getDistance().in(Meters));

	// Intake Commands
	Command Intake_Open		= mIntake.intakeCommand();
	Command Intake_Halt		= mIntake.haltCommand();
	Command Intake_Close	= mIntake.closeCommand();
	Command Intake_Pulse	= mIntake.agitateCommand();
	Command Intake_Calibrate= mIntake.calibrateCommand();

	// Firing Commands
	// YFH = Yaw Flywheels Hood
	Command FH_Static_Pos   = mRanger.setStaticCommand();
	Command AutoYFH_Enable  = mRanger.enableContinousCommand ().andThen(mLemonLime.getEnableCommand ());
	Command AutoYFH_Disable = mRanger.disableContinousCommand().andThen(mLemonLime.getDisableCommand());
	Command FH_Downrange    = mRanger.setPassingCommand();
	Command FH_Stop			= mRanger.setDisabledCommand();

	// Feeder Commands
	Command Feeder_Reverse  = mFloor.reverseCommand().alongWith(mFeeder.reverseCommand());
	Command Feeder_Forward	= mFloor.feedCommand().alongWith(mFeeder.feedCommand());
	Command Feeder_Stop		= mFloor.haltCommand().alongWith(mFeeder.haltCommand());
	Command Feeder_Forward_and_Pulse = mFloor.feedCommand().alongWith(mFeeder.feedCommand())
										.andThen(Commands.waitSeconds(1)).andThen(mIntake.agitateCommand());
	Command Launcher_Backfeed  = 	mFloor.reverseCommand().alongWith(mFeeder.reverseCommand());
		
	public RobotContainer() {
		configureBindings();
		configureNamedCommands();
		driveAdapter.setVehicleYawSupplier(()->drivebase.getHeading().getRadians());

		driveAdapter.setFieldOriented(false);
		drivebase.resetOdometry(Constants.AutonConstants.DefaultPose.selected.getAsPose2d());
		drivebase.attachAutonSteeringIntercept(()->mLemonLime.getEnabled(), mLemonLime.getAutonInterceptRequest());
	}

	private void configureNamedCommands(){
		NamedCommands.registerCommand("Placeholder Command", Commands.print(" <!> Placeholder Command Triggered"));
		NamedCommands.registerCommand("Placeholder Command II", Commands.print(" <!> Placeholder Command II Triggered"));
		NamedCommands.registerCommand("Do Nothing", Commands.none());
		NamedCommands.registerCommand("Auto YFH Enable",AutoYFH_Enable);
		NamedCommands.registerCommand("Auto YFH Disable",AutoYFH_Disable);

		NamedCommands.registerCommand("Intake Open",Intake_Open);
		NamedCommands.registerCommand("Intake Disable",Intake_Halt);
		NamedCommands.registerCommand("Intake Close",Intake_Close);
		NamedCommands.registerCommand("Intake Pulse",Intake_Pulse);
		NamedCommands.registerCommand("Intake Calibrate",Intake_Calibrate);

		NamedCommands.registerCommand("Spool Up for Hopper", FH_Static_Pos);
		NamedCommands.registerCommand("Spool Up for Passing", FH_Downrange);
		NamedCommands.registerCommand("Spool Down", FH_Stop);
		NamedCommands.registerCommand("Feeder Forward", Feeder_Forward);
		NamedCommands.registerCommand("Feeder Reverse", Feeder_Reverse);
		NamedCommands.registerCommand("Feeder Stop", Feeder_Stop);

		NamedCommands.registerCommand("enable steering shit", mLemonLime.getEnableCommand());
	}

	private void configureBindings() {
		//Pass vision data to the swerve drive system
		mLimelightPositioner.setDefaultCommand(updateVisionCommand());

		// Swerve drive calibration and configuration.
		drivebase.setDefaultCommand(
			drivebase.driveCommand(
				driveAdapter::getDriveY, driveAdapter::getDriveX, 
				()->(driveAdapter.getDriveR() + mLemonLime.getVisualJoyStick()),
			false));
		control.d_LSB().and(control.d_RSB()).onTrue		(Commands.runOnce(()->setMotorBrake(true)));
		control.d_LSB().and(control.d_RSB()).onFalse	(Commands.runOnce(()->setMotorBrake(false)));
		
		// Intake Control
		control.d_L2().onTrue		(Intake_Open ); // Open & Activate Intake
		control.d_L2().onFalse		(Intake_Halt ); // Open & Disable Intake
		control.d_povDown().onTrue	(Intake_Close); // Close & Disable Intake
		control.d_povRight().onTrue	(Intake_Pulse); // Pulse Intake (as adjetator)
		control.d_menu().onTrue		(Intake_Calibrate); // zero the intake via the limit switch

		// Fire Control
		control.d_R2().onTrue		(AutoYFH_Enable	); // Fire
		control.d_R2().onFalse		(AutoYFH_Disable); // Fire
		control.d_R1().onTrue		(FH_Downrange	); // Fire
		control.d_R1().onFalse		(FH_Stop		); // Stop Firing
		control.d_cross().onTrue	(Launcher_Backfeed); // Backfeed
		control.d_circle().or(control.d_R2()).onTrue	(Feeder_Forward_and_Pulse); // Backfeed

		control.d_circle().or(control.d_cross()).or(control.d_R2()).onFalse(Feeder_Stop);
	} 

	public Command getAutonomousCommand() {
		System.out.println("Autonomous Command Loaded");
		return AutoBuilder.buildAuto(Constants.AutonConstants.AutonName)
			.andThen(
				mLemonLime.getDisableCommand()
				.alongWith(mRanger.disableContinousCommand())
			);
	}

	public void setMotorBrake(boolean brake) { 
		drivebase.setMotorBrake(brake);
		if(brake) drivebase.lock();
	}

	private Command updateVisionCommand() {
        return mLimelightPositioner.run(() -> {
            final Pose2d currentRobotPose = drivebase.getPose();
            final Optional<LimelightPositioning.Measurement> measurement = mLimelightPositioner.getMeasurement(currentRobotPose);
            measurement.ifPresent(m -> {
                drivebase.getSwerveDrive().addVisionMeasurement(
                    m.poseEstimate.pose, 
                    m.poseEstimate.timestampSeconds,
                    m.standardDeviations
                );
            });
			mLimelightPositioner.printMegaTagToDashboard();
        })
        .ignoringDisable(true);
    }
}