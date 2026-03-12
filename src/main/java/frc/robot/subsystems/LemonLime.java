package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.stock.SwerveSubsystem;

public class LemonLime extends SubsystemBase {
	private final SwerveSubsystem swerve;
	private boolean enabled = false;

	public LemonLime(SwerveSubsystem drivebase) {
		swerve = drivebase;
		swerve.attachAutonSteeringIntercept(()->enabled,getAutonInterceptRequest());
	}

	public Command toggleVisionDriving() {return runOnce(() -> setEnabled(!this.enabled));}
	public Command getEnableCommand   () {return runOnce(() -> setEnabled(true  ));}
	public Command getDisableCommand  () {return runOnce(() -> setEnabled(false ));}
	public void setEnabled(boolean enable) { this.enabled = enable; }
	public boolean getEnabled(){ return enabled; }

	@Override
	public void periodic(){
		SmartDashboard.putNumber("Lemon Angular Offset", getAngularOffset().getDegrees());
		SmartDashboard.putNumber("Lemon Linear Offset", getDistance().in(Meters));
	}
	
	public Alliance getAlliance(){ return DriverStation.getAlliance().orElse(Alliance.Red); }

	private double clamp(double x, double a, double b){ return x<a ? a : x>b ? b : x; }

	public Pose2d getTartetPose(){
		return getAlliance().equals(Alliance.Red) ? 
			new Pose2d(Meters.of(11.917), Meters.of(4.029), new Rotation2d()) : // red
			new Pose2d(Meters.of( 4.623), Meters.of(4.029), new Rotation2d()) ; // blue
	}

	public Rotation2d getAngularOffset(){
		return getTartetPose().relativeTo(swerve.getPose()).getTranslation().getAngle();
	}

	public Distance getDistance(){
		return Meters.of(swerve.getPose().getTranslation().getDistance(getTartetPose().getTranslation()));
	}

	public double getVisualJoyStick(){
		double n = clamp(getAngularOffset().getDegrees(),-50,50);
		n=Math.copySign(Math.abs(n)*.014+.29, n);
		//only return calculated value if aiming enabled
		return enabled && Math.abs(n)>.295 ? n : 0;
	}

	public DoubleSupplier getAutonInterceptRequest(){
		return ()->clamp(getAngularOffset().getDegrees(),-50,50)*.1;//0.0; // this is in RPS
	}
}