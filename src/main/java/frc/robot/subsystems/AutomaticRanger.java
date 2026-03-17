package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.wcp.Hood;
import frc.robot.subsystems.wcp.Shooter;

public class AutomaticRanger extends SubsystemBase{

	private final Shooter shooter;
	private final Hood hood;
	private final DoubleSupplier range;
	private boolean active = false;

	private final double[] rpm = {
		3000,3000,3000,//0
		3500,//20
		3550,//40
		3550,//60
		3700,//80
		3900,//100
		4100,//120
		4350,//140
		4400,//160
		4500,//180
	};
	private final double[] height = {
		0,0,0, //0
		.1, //20
		.15, //40
		.23,//60
		.3, //80
		.35,  //100
		.39,  //120
		.42, //140
		.45,  //160
		.48,   //180
	};
	
	public AutomaticRanger(Shooter s, Hood h, DoubleSupplier r){
		super();
		shooter=s;
		hood=h;
		range=r;
		SmartDashboard.putNumber("Auto RPM", 3000);
		SmartDashboard.putNumber("Auto Height", .19);
	}

	@Override
	public void periodic(){
		if(active){ setAutomatic(); }
	}

	private double lerp(double[] arr, double n){
		if(n<0) n=0;
		if(n>arr.length) n=arr.length;
		int nn = (int) n;
		return arr[nn] * (nn-n+1) + arr[nn+1] * (n-nn);
	}

	public void setRaw(double shooter_rpm, double hood_height){
		shooter.setRPM(shooter_rpm);
		hood.setPosition(hood_height);
		SmartDashboard.putNumber("FH Shooter RPM", shooter_rpm);
		SmartDashboard.putNumber("FH Hood Height", hood_height);
	}

	public void setDisabled(){ setRaw(0, 0); }
	public void setPassing(){ setRaw(3000, 1); }
	public void setStatic(){ setRaw(3500, .19); }
	public void setFromDashboard(){
		setRaw(SmartDashboard.getNumber("Auto RPM", 3000),SmartDashboard.getNumber("Auto Height", .19));
	}
		public void setAutomatic(){
		double r = range.getAsDouble();
		setRaw(lerp(rpm,r*2), lerp(height, r*2));
	}

	public void enableContinous (){ active =  true  ; }
	public void disableContinous(){ active =  false ; setDisabled();}
	public void toggleContinous (){ active = !active; if(!active)setDisabled(); }

	public boolean getIsContinousEnabled(){ return active   ; }
	
	public Command setDisabledCommand		(){ return runOnce(()->setDisabled		()); }
	public Command setStaticCommand			(){ return runOnce(()->setStatic		()); }
	public Command setPassingCommand		(){ return runOnce(()->setPassing		()); }
	public Command setAutomaticCommand		(){ return runOnce(()->setFromDashboard	()); }
	public Command enableContinousCommand	(){ return runOnce(()->enableContinous	()); }
	public Command disableContinousCommand	(){ return runOnce(()->disableContinous	()); }
	public Command toggleContinousCommand	(){ return runOnce(()->toggleContinous	()); }
}
