package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.stock.LimelightHelpers;
import frc.robot.subsystems.stock.SwerveSubsystem;
import frc.robot.subsystems.stock.LimelightHelpers.PoseEstimate;

public class LemonLime extends SubsystemBase {
	private final SwerveSubsystem swerve;
	private boolean enabled = false;

	public LemonLime(SwerveSubsystem drivebase) {
		swerve = drivebase;
		swerve.attachAutonSteeringIntercept(()->enabled,getAutonInterceptRequest());
	}

	public Command toggleVisionDriving() {
		return runOnce(() -> setEnabled(!this.enabled));
	}

	public void setEnabled(boolean enable) {
		this.enabled = enable;
	}

	public boolean getEnabled(){ return enabled; }

	@Override
	public void periodic() {
		// if(enabled){
		// 	PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
		// 	if (estimate != null && estimate.pose != null) {
		// 		swerve.getSwerveDrive().addVisionMeasurement(
		// 			estimate.pose,
		// 			estimate.timestampSeconds,
		// 			null
		// 		);
		// 	}
		// }
		SmartDashboard.putNumber("Lemon Angle", getAngularOffset().getDegrees());
		SmartDashboard.putNumber("Lemon Joystict", getVisualJoyStick());
		SmartDashboard.putString("Lemon Coords", swerve.getPose().getX()+", "+swerve.getPose().getY());
	}
	
	public Alliance getAlliance(){ return DriverStation.getAlliance().get(); }

	private double clamp(double x, double a, double b){
		return x<a ? a : x>b ? b : x;
	}

	public Rotation2d getAngularOffset(){
		Pose2d target = getAlliance()!=null && getAlliance().equals(Alliance.Red) ? 
			new Pose2d(Inches.of(651.22-158.84), Inches.of(181.46), new Rotation2d(0)) : // Red Hub (default)
			new Pose2d(Inches.of(158.84), Inches.of(181.46), new Rotation2d(0)) ; // Blue Hub

		Pose2d self = swerve.getPose();
		target = target.relativeTo(self);

		return target.getTranslation().getAngle();
	}

	public double getVisualJoyStick(){
		return clamp(getAngularOffset().times(-.5).getTan(), -.1, .1);
	}

	public DoubleSupplier getAutonInterceptRequest(){
		return ()->0.0; // this is in RPS
	}
}