package frc.robot.subsystems;

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
    }

    public Command enableVisionDriving() {
        return runOnce(() -> setEnabled(true));
    }

    public Command disableVisionDriving() {
        return runOnce(() -> setEnabled(false));
    }

    public void setEnabled(boolean enable) {
        this.enabled = enable;
    }

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
}