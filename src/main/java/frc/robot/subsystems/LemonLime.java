package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.stock.LimelightHelpers;
import frc.robot.subsystems.stock.SwerveSubsystem;
import frc.robot.subsystems.wcp.Limelight;
import frc.robot.subsystems.stock.LimelightHelpers.PoseEstimate;

public class LemonLime extends SubsystemBase {
	private static final String LIMELIGHT_NAME = "limelight";

	private final SwerveSubsystem swerve;
	private boolean enabled = false;
    private final NetworkTable telemetryTable;
    private final StructPublisher<Pose2d> posePublisher;


	public LemonLime(SwerveSubsystem drivebase) {
		swerve = drivebase;
		swerve.attachAutonSteeringIntercept(()->enabled,getAutonInterceptRequest());
		this.telemetryTable = NetworkTableInstance.getDefault().getTable("SmartDashboard/" + LIMELIGHT_NAME);
        this.posePublisher = telemetryTable.getStructTopic("Estimated Robot Pose", Pose2d.struct).publish();
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
		if(enabled){
			final Pose2d currentRobotPose = swerve.getPose();
            final Optional<Measurement> measurement = getMeasurement(currentRobotPose);
            measurement.ifPresent(m -> {
                swerve.getSwerveDrive().addVisionMeasurement(
                    m.poseEstimate.pose, 
                    m.poseEstimate.timestampSeconds,
                    m.standardDeviations
                );
            });
		}
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

	public Optional<Measurement> getMeasurement(Pose2d currentRobotPose) {
        LimelightHelpers.SetRobotOrientation(LIMELIGHT_NAME, currentRobotPose.getRotation().getDegrees(), 0, 0, 0, 0, 0);

        final PoseEstimate poseEstimate_MegaTag1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(LIMELIGHT_NAME);
        final PoseEstimate poseEstimate_MegaTag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LIMELIGHT_NAME);
        if (
            poseEstimate_MegaTag1 == null 
                || poseEstimate_MegaTag2 == null
                || poseEstimate_MegaTag1.tagCount == 0
                || poseEstimate_MegaTag2.tagCount == 0
        ) {
            return Optional.empty();
        }

        // Combine the readings from MegaTag1 and MegaTag2:
        // 1. Use the more stable position from MegaTag2
        // 2. Use the rotation from MegaTag1 (with low confidence) to counteract gyro drift
        poseEstimate_MegaTag2.pose = new Pose2d(
            poseEstimate_MegaTag2.pose.getTranslation(),
            poseEstimate_MegaTag1.pose.getRotation()
        );
        final Matrix<N3, N1> standardDeviations = VecBuilder.fill(0.1, 0.1, 10.0);

        posePublisher.set(poseEstimate_MegaTag2.pose);

        return Optional.of(new Measurement(poseEstimate_MegaTag2, standardDeviations));
    }

    public static class Measurement {
        public final PoseEstimate poseEstimate;
        public final Matrix<N3, N1> standardDeviations;

        public Measurement(PoseEstimate poseEstimate, Matrix<N3, N1> standardDeviations) {
            this.poseEstimate = poseEstimate;
            this.standardDeviations = standardDeviations;
        }
    }
}