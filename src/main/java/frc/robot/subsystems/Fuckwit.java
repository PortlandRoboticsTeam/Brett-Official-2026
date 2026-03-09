package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.wcp.Hood;
import frc.robot.subsystems.wcp.Shooter;

public class Fuckwit extends SubsystemBase{

	private final Shooter shooter;
	private final Hood hood;
	private final DoubleSupplier range;
	private boolean active = false;

	private final double[] rpm = {0.0,1.0};
	private final double[] height = {0.0,1.0};
	
	public Fuckwit(Shooter s, Hood h, DoubleSupplier r){
		super();
		shooter=s;
		hood=h;
		range=r;
	}

	@Override
	public void periodic(){
		if(active){
			setAutomatic();
		}
	}

	private double lerp(double[] arr, double n){
		if(n<0) n=0;
		if(n>arr.length) n=arr.length;
		int nn = (int) n;
		return arr[nn] * (nn-n+1) + arr[nn+1] * (n-nn);
	}

	public void setRaw(double hood_height, double shooter_rpm){
		shooter.setRPM(shooter_rpm); 
		hood.setPosition(hood_height);
	}

	public void setDisabled(){ setRaw(0, 0); }
	public void setStatic(){ setRaw(3500, .19); }
	public void setAutomatic(){ double r = range.getAsDouble(); setRaw(lerp(rpm,r), lerp(height, r)); }
	public void enableContinous  (){ active =  true  ; }
	public void disableContinous (){ active =  false ; }
	public void toggleContinous  (){ active = !active; }
	public boolean getIsContinous(){ return active   ; }
	
	public Command setDisabledCommand		(){ return runOnce(()->setDisabled		()); }
	public Command setStaticCommand			(){ return runOnce(()->setStatic		()); }
	public Command setAutomaticCommand		(){ return runOnce(()->setAutomatic		()); }
	public Command enableContinousCommand	(){ return runOnce(()->enableContinous	()); }
	public Command disableContinousCommand	(){ return runOnce(()->disableContinous	()); }
	public Command toggleContinousCommand	(){ return runOnce(()->toggleContinous	()); }
}
