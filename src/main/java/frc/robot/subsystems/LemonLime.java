package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.stock.LimelightHelpers;
import frc.robot.subsystems.stock.SwerveSubsystem;
import frc.robot.subsystems.stock.LimelightHelpers.PoseEstimate;
import frc.robot.Constants;
import frc.robot.commands.AimAndDriveCommand;

public class LemonLime extends SubsystemBase {
	private final SwerveSubsystem swerve;
	private boolean enabled = false;
	private final PIDController driveStickPID = new PIDController(.04, .002, .02);
	private Pose2d 	pose_p	= new Pose2d(), 
					pose	= new Pose2d();

	public LemonLime(SwerveSubsystem drivebase) {
		swerve = drivebase;
		driveStickPID.reset();
		swerve.attachAutonSteeringIntercept(()->enabled,getAutonInterceptRequest());
		driveStickPID.enableContinuousInput(-180, 180);
	}

	public Command toggleVisionDriving() {return runOnce(() -> setEnabled(!this.enabled));}
	public Command getEnableCommand   () {return runOnce(() -> setEnabled(true  ));}
	public Command getDisableCommand  () {return runOnce(() -> setEnabled(false ));}

	public void setEnabled(boolean enable) { this.enabled = enable; }
	
	public boolean getEnabled(){ return enabled; }

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Lemon Angle", getAngularOffset().getDegrees());
		SmartDashboard.putNumber("Lemon Joystick", getVisualJoyStick());
		SmartDashboard.putString("Lemon Coords", swerve.getPose().getX()+", "+swerve.getPose().getY());

		ChassisSpeeds velo = swerve.getRobotVelocity();
		pose=swerve.getPose();

		PoseEstimate[] p = {
			LimelightHelpers.getBotPoseEstimate_wpiBlue(Constants.Ports.FORWARD_LIMELIGHT),
			LimelightHelpers.getBotPoseEstimate_wpiRed(Constants.Ports.FORWARD_LIMELIGHT),
			LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.Ports.FORWARD_LIMELIGHT),
			LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(Constants.Ports.FORWARD_LIMELIGHT)
		};

		for(int i=0; i<4; i++){
			SmartDashboard.putString("Megatag Read "+i,
				"is MT2: "+p[i].isMegaTag2+
				";\nPose: "+p[i].pose.toString()+
				";\nArea: "+p[i].avgTagArea+
				";\nDist: "+p[i].avgTagDist+
				";\nCount: "+p[i].tagCount+
				";\nLatency: "+p[i].latency+
				";\n2str: "+p[i].toString()
			);
		}
	}
	
	public Alliance getAlliance(){ return DriverStation.getAlliance().orElse(Alliance.Red); }

	private double clamp(double x, double a, double b){ return x<a ? a : x>b ? b : x; }

	public Pose2d getTartetPose(){
		return getAlliance().equals(Alliance.Red) ? 
			new Pose2d(Inches.of(158.84), Inches.of(181.46), new Rotation2d(0)) : // Blue Hub (default)
			new Pose2d(Inches.of(651.22-158.84), Inches.of(181.46), new Rotation2d(0)) ; // Red Hub
	}

	public Rotation2d getAngularOffset(){
		return getTartetPose().relativeTo(pose).getTranslation().getAngle();
	}

	// public Distance getRangeRequest(){
	// 	// if(!enabled) return Meters.of(1);
	// 	ChassisSpeeds velo = swerve.getFieldVelocity();
	// 	Transform2d dx = new Pose2d(new Translation2d(velo.vxMetersPerSecond,velo.vyMetersPerSecond),new Rotation2d());
	// 	Rotation2d da = pose.getRotation().minus(pose_p.getRotation()).times(-50);

	// 	Pose2d rel = getTartetPose().relativeTo(swerve.getPose())
	// 					.rotateAround(new Translation2d(), da)
	// 					.plus(dx);
	// 	double d = rel.getTranslation().getNorm();
	// 	return Meters.of( d<1 ? 1 : d );
	// }

	public Rotation2d getAngleRequest(){
		// if(!enabled) return new Rotation2d();
		Transform2d dx = pose.minus(pose_p).times(-50);
		Rotation2d da = pose.getRotation().minus(pose_p.getRotation()).times(-50);

		Pose2d rel = getTartetPose()
						.relativeTo(pose)
						.rotateAround(new Translation2d(), da)
						.plus(dx);
		
		double d = rel.getTranslation().getNorm();
		return d<1 ? new Rotation2d() : rel.getTranslation().getAngle();
	}

	public double getVisualJoyStick(){
		double n = clamp(driveStickPID.calculate(swerve.getHeading().getDegrees(),AimAndDriveCommand.getDirectionToHub(swerve).getDegrees()),-1,1);
		n=Math.copySign(Math.abs(n)*.7+.3, n);
		return enabled ? n : 0;

	}
	public DoubleSupplier getAutonInterceptRequest(){
		return ()->0.0; // this is in RPS
	}
}