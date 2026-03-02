package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
		if (!enabled) return; // skip vision updates when disabled

		PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
		if (estimate != null && estimate.pose != null) {
			swerve.getSwerveDrive().addVisionMeasurement(
				estimate.pose,
				estimate.timestampSeconds,
				null
			);
		}
	}
	
	public Alliance getAlliance(){ return DriverStation.getAlliance().get(); }

	private double clamp(double x, double a, double b){
		return x<a ? a : x>b ? b : x;
	}

	public double getAngularOffset(){
		Pose2d target = getAlliance()!=null && getAlliance().equals(Alliance.Red) ? 
			new Pose2d() : // Red Hub (default)
			new Pose2d() ; // Blue Hub

		Pose2d self = swerve.getPose();
		target = target.relativeTo(self).rotateBy(self.getRotation().times(-1));

		return clamp(target.getTranslation().getAngle().times(.5).getTan(),-1,1);
	}

	public DoubleSupplier getAutonInterceptRequest(){
		return ()->0.0; // this is in RPS
	}
}