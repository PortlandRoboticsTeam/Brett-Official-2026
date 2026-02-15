package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import javax.sound.midi.ControllerEventListener;

import frc.robot.Constants.OperatorConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;

public class DriveControllerAdapter extends SubsystemBase{
	
	double intx = 0, inty = 0, intr = 0;

	final ControllerHandler ctrl;
	DoubleSupplier yaw = () -> 0;
	boolean isFieldOriented = OperatorConstants.DEFAULT_IS_FIELD_ORIENTED;

	public DriveControllerAdapter(ControllerHandler c){
		super();
		ctrl=c;
	}

	public void setVehicleYawSupplier(DoubleSupplier newYawSupplier){ yaw = newYawSupplier; }

	public boolean    setFieldOriented (boolean newval) { isFieldOriented = newval; return isFieldOriented;}
	public boolean    getFieldOriented ( ) { return isFieldOriented;}
	public boolean toggleFieldOriented ( ) { isFieldOriented = !isFieldOriented; return isFieldOriented;}

	@Override
	public void periodic(){
		double x=ctrl.d_leftX()*.7, y=ctrl.d_leftY()*.7, r=ctrl.d_rightX();
		double a=yaw.getAsDouble(), db=OperatorConstants.DEADBAND;

		x = x>db ? x-db : x<-db ? x+db : 0;
		y = y>db ? y-db : y<-db ? y+db : 0;
		r = r>db ? r-db : r<-db ? r+db : 0;

		if(isFieldOriented){
			double temp = x*Math.cos(a) - y*Math.sin(a);
			y = x*Math.sin(a) + y*Math.cos(a);
			x = temp;
		}

		intx = intx*(1-OperatorConstants. LINEAR_CONTROL_APPROACH_RATE) + x*OperatorConstants. LINEAR_CONTROL_APPROACH_RATE;
		inty = inty*(1-OperatorConstants. LINEAR_CONTROL_APPROACH_RATE) + y*OperatorConstants. LINEAR_CONTROL_APPROACH_RATE;
		intr = intr*(1-OperatorConstants.ANGULAR_CONTROL_APPROACH_RATE) + r*OperatorConstants.ANGULAR_CONTROL_APPROACH_RATE;
	}

	public double getDriveX(){return intx;}
	public double getDriveY(){return inty;}
	public double getDriveR(){return intr;}
}
