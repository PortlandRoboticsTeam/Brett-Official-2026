package frc.robot.subsystems.wcp;

import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.stock.LimelightHelpers;
import frc.robot.subsystems.stock.LimelightHelpers.PoseEstimate;

public class LimelightPositioning extends SubsystemBase {
	private final String name;
	private final NetworkTable telemetryTable;
	private final StructPublisher<Pose2d> posePublisher;

	public LimelightPositioning(String name) {
		this.name = name;
		this.telemetryTable = NetworkTableInstance.getDefault().getTable("SmartDashboard/" + name);
		this.posePublisher = telemetryTable.getStructTopic("Estimated Robot Pose", Pose2d.struct).publish();
	}

	public Optional<Measurement> getMeasurement(Pose2d currentRobotPose) {
		LimelightHelpers.SetRobotOrientation(name, currentRobotPose.getRotation().getDegrees(), 0, 0, 0, 0, 0);

		final PoseEstimate poseEstimate_MegaTag1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
		final PoseEstimate poseEstimate_MegaTag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
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

	public void printMegaTagToDashboard(){
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

	public static class Measurement {
		public final PoseEstimate poseEstimate;
		public final Matrix<N3, N1> standardDeviations;

		public Measurement(PoseEstimate poseEstimate, Matrix<N3, N1> standardDeviations) {
			this.poseEstimate = poseEstimate;
			this.standardDeviations = standardDeviations;
		}
	}
}
