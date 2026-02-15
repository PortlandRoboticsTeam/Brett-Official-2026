package frc.robot.subsystems;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ControllerHandler extends SubsystemBase {

	/** NOTES:
	 * 
	 * all buttons read target controller OR other controller pretending
	 * that means onTrue triggers when either activate, and onfalse activates when both are inactive
	 * 
	 * Joysticks are directly overwritten by the controller pretending
	 * this is to avoid conflicts of interests and odd corner-cases with trigger axes
	 * 
	 * rumble packs are disabled whenever the swap key is pressed or released to prevent confusion
	**/

	private final CommandPS5Controller dcc, hcc; // driver, helper
	private final int swapButton;
	private boolean d_swap=false, h_swap=false, one_handed=false;
	public ControllerHandler(int swapButtonID, int driverPort, int helperPort){
		super();
		swapButton = swapButtonID;
		dcc = new CommandPS5Controller(driverPort);
		hcc = new CommandPS5Controller(helperPort);
		dcc.button(swapButton).onTrue (runOnce(() -> { d_swap=true ; dcc.setRumble(RumbleType.kBothRumble, 0); }));
		dcc.button(swapButton).onFalse(runOnce(() -> { d_swap=false; dcc.setRumble(RumbleType.kBothRumble, 0); }));
		hcc.button(swapButton).onTrue (runOnce(() -> { h_swap=true ; dcc.setRumble(RumbleType.kBothRumble, 0); }));
		hcc.button(swapButton).onFalse(runOnce(() -> { h_swap=false; dcc.setRumble(RumbleType.kBothRumble, 0); }));
	}

	public ControllerHandler(){this(5,0,1);}
	public boolean setOneHanded(boolean newval){ one_handed=newval; return one_handed; }
	public boolean getOneHanded(){ return one_handed; }


	public double d_getAxis(int axis) {
		return h_swap ? hcc.getRawAxis(axis) : dcc.getRawAxis(axis);
	}
	public double h_getAxis(int axis) {
		return d_swap ? dcc.getRawAxis(axis) : hcc.getRawAxis(axis);
	}

	public Trigger d_button(int button_id){
		if(button_id == swapButton){
			throw new Error(" >>> Illegal Button Binding: Attempted to bind a control to button \""+button_id+"\" on driver controller when that button is reserved for the swap button.");
		}
		return dcc.button(swapButton).negate().and(dcc.button(button_id)).or(
			hcc.button(swapButton).and(hcc.button(button_id)));
	}
	public Trigger h_button(int button_id){
		if(button_id == swapButton){
			throw new Error(" >>> Illegal Button Binding: Attempted to bind a control to button \""+button_id+"\" on helper controller when that button is reserved for the swap button.");
		}
		return hcc.button(swapButton).negate().and(hcc.button(button_id)).or(
			dcc.button(swapButton).and(dcc.button(button_id).or(()->one_handed)));
	}

	public Trigger d_pov(int angle){
		return dcc.button(swapButton).negate().and(dcc.pov(angle)).or(
			hcc.button(swapButton).and(hcc.pov(angle)));
	}
	public Trigger h_pov(int angle){
		return hcc.button(swapButton).negate().and(hcc.pov(angle)).or(
			dcc.button(swapButton).and(dcc.pov(angle)));
	}

	public boolean driverConnected(){ return dcc.isConnected(); }
	public boolean helperConnected(){ return hcc.isConnected(); }

	public void driverRumble(RumbleType type, double value){ 
		if(!d_swap) dcc.setRumble(type, value); 
		if( h_swap) hcc.setRumble(type, value); 
	}
	public void helperRumble(RumbleType type, double value){ 
		if( d_swap) dcc.setRumble(type, value); 
		if(!h_swap) hcc.setRumble(type, value); 
	}

	public double d_leftX (){ return d_getAxis(0); }
	public double d_leftY (){ return d_getAxis(1); }
	public double d_rightX(){ return d_getAxis(2); }
	public double d_rightY(){ return d_getAxis(5); }
	public double d_L2axis(){ return d_getAxis(3); }
	public double d_R2axis(){ return d_getAxis(4); }

	public double h_leftX (){ return h_getAxis(0); }
	public double h_leftY (){ return h_getAxis(1); }
	public double h_rightX(){ return h_getAxis(2); }
	public double h_rightY(){ return h_getAxis(5); }
	public double h_L2axis(){ return h_getAxis(3); }
	public double h_R2axis(){ return h_getAxis(4); }

	public Trigger d_square   (){ return d_button( 1); }
	public Trigger d_cross    (){ return d_button( 2); }
	public Trigger d_circle   (){ return d_button( 3); }
	public Trigger d_triangle (){ return d_button( 4); }
	public Trigger d_L1       (){ return d_button( 5); }
	public Trigger d_R1       (){ return d_button( 6); }
	public Trigger d_L2       (){ return d_button( 7); }
	public Trigger d_R2       (){ return d_button( 8); }
	public Trigger d_blink    (){ return d_button( 9); }
	public Trigger d_menu     (){ return d_button(10); }
	public Trigger d_LSB      (){ return d_button(11); }
	public Trigger d_RSB      (){ return d_button(12); }
	public Trigger d_PSButton (){ return d_button(11); }
	public Trigger d_Touch    (){ return d_button(12); }
	public Trigger d_Mute     (){ return d_button(11); }

	public Trigger h_square   (){ return h_button( 1); }
	public Trigger h_cross    (){ return h_button( 2); }
	public Trigger h_circle   (){ return h_button( 3); }
	public Trigger h_triangle (){ return h_button( 4); }
	public Trigger h_L1       (){ return h_button( 5); }
	public Trigger h_R1       (){ return h_button( 6); }
	public Trigger h_L2       (){ return h_button( 7); }
	public Trigger h_R2       (){ return h_button( 8); }
	public Trigger h_blink    (){ return h_button( 9); }
	public Trigger h_menu     (){ return h_button(10); }
	public Trigger h_LSB      (){ return h_button(11); }
	public Trigger h_RSB      (){ return h_button(12); }
	public Trigger h_PSButton (){ return h_button(11); }
	public Trigger h_Touch    (){ return h_button(12); }
	public Trigger h_Mute     (){ return h_button(11); }
	
	public Trigger d_povUp        (){ return d_pov(0); }
	public Trigger d_povUpRight   (){ return d_pov(45); }
	public Trigger d_povRight     (){ return d_pov(90); }
	public Trigger d_povDownRight (){ return d_pov(135); }
	public Trigger d_povDown      (){ return d_pov(180); }
	public Trigger d_povDownLeft  (){ return d_pov(225); }
	public Trigger d_povLeft      (){ return d_pov(270); }
	public Trigger d_povUpLeft    (){ return d_pov(315); }
	public Trigger d_povCenter    (){ return d_pov(-1); }

	public Trigger h_povUp        (){ return h_pov(0); }
	public Trigger h_povUpRight   (){ return h_pov(45); }
	public Trigger h_povRight     (){ return h_pov(90); }
	public Trigger h_povDownRight (){ return h_pov(135); }
	public Trigger h_povDown      (){ return h_pov(180); }
	public Trigger h_povDownLeft  (){ return h_pov(225); }
	public Trigger h_povLeft      (){ return h_pov(270); }
	public Trigger h_povUpLeft    (){ return h_pov(315); }
	public Trigger h_povCenter    (){ return h_pov(-1); }
}
