package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.stock.LimelightHelpers;
import frc.robot.subsystems.stock.SwerveSubsystem;

public class LemonLime extends SubsystemBase{
    final SwerveSubsystem swerve;
    public LemonLime(SwerveSubsystem drivebase){
        super();
        swerve = drivebase;
        LimelightHelpers.getBotPoseEstimate_wpiBlue("");
    }

    @Override
    public void periodic(){
        var o = LimelightHelpers.getBotPoseEstimate_wpiBlue("");
        swerve.resetOdometry(o.pose);
    }
}
