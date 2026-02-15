// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.RPM;
import edu.wpi.first.units.measure.AngularVelocity;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

	public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
	public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
	public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
	public static final double MAX_SPEED  = Units.feetToMeters(14.5);
	// Maximum speed of the robot in meters per second, used to limit acceleration.
	public static class KrakenX60 {
		public static final AngularVelocity kFreeSpeed = RPM.of(6000);
	}

 public static final class AutonConstants
 {

   public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
   public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
 }

	public static final class DrivebaseConstants
	{

		// Hold time on motor brakes when disabled
		public static final double WHEEL_LOCK_TIME = 10; // seconds
	}

	public static class OperatorConstants
	{

		// Joystick Deadband
		public static final double 
			DEADBAND         = 0.2,  
			LEFT_Y_DEADBAND  = 0.15,  
			RIGHT_X_DEADBAND = 0.1,  
			TURN_CONSTANT    = 6;
		
		public static final boolean DEFAULT_IS_FIELD_ORIENTED = false;

		public static final double 
			LINEAR_CONTROL_APPROACH_RATE  = .2,
			ANGULAR_CONTROL_APPROACH_RATE = .2;
	}

	public final class Ports {
		// CAN Buses
		public static final CANBus kRoboRioCANBus = new CANBus("rio");
		public static final CANBus kCANivoreCANBus = new CANBus("main");

		// Talon FX IDs
		public static final int kIntakePivot 		= 10 + 10;
		public static final int kIntakeRollers 	= 11 + 10;
		public static final int kFloor 					= 12 + 10;
		public static final int kFeeder 				= 13 + 10;
		public static final int kShooterLeft 		= 14 + 10;
		public static final int kShooterMiddle 	= 15 + 10;
		public static final int kShooterRight 	= 16 + 10;
		public static final int kHanger 				= 18 + 10;

		// PWM Ports
		public static final int kHoodLeftServo 	= 3;
		public static final int kHoodRightServo = 4;
	}
}
