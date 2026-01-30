package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import frc.robot.Constants.OperatorConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;

public class Controller extends SubsystemBase{
	
	double intx = 0, inty = 0, intr = 0;

	final CommandPS5Controller driver, helper;
	DoubleSupplier yaw = () -> 0;
	boolean isFieldOriented = OperatorConstants.DEFAULT_IS_FIELD_ORIENTED;

	public Controller(int driverPort, int helperPort){
		super();
		driver = new CommandPS5Controller(driverPort);
		helper = new CommandPS5Controller(helperPort);
	}

	public void setYawSupplier(DoubleSupplier newYawSupplier){ yaw = newYawSupplier; }

	public boolean    setFieldOriented (boolean newval) { isFieldOriented = newval; return isFieldOriented;}
	public boolean    getFieldOriented ( ) { return isFieldOriented;}
	public boolean toggleFieldOriented ( ) { isFieldOriented = !isFieldOriented; return isFieldOriented;}

	public CommandPS5Controller driver(){return driver;}
	public CommandPS5Controller helper(){return helper;}

	@Override
	public void periodic(){
		double x=driver.getLeftX()*.7, y=driver.getLeftY()*.7, r=driver.getRightX();
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
