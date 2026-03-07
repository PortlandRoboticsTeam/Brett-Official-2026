package frc.robot.subsystems;

import java.io.IOException;
import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import com.pathplanner.lib.path.PathPlannerPath;

public class AutoCompiler{
	private static Command auto;

	public static void SetUpCompiler(){
		SmartDashboard.putString("Auto Alert", "Ready...\nNo auto loaded");
		SmartDashboard.putBoolean("Auto Loaded", false);
	}

	public static void loadBlankAuto(){
		auto=Commands.none();
		SmartDashboard.putBoolean("Auto Loaded", true);
		SmartDashboard.putString("Auto Alert", "Load Success\nBlank auto loaded");
	}
	
	public static void loadAutonomousProgram(String name, boolean flipSide){
		PathPlannerPath p = null;
		try{
			p = PathPlannerPath.fromPathFile(name);
			SmartDashboard.putString("Auto Alert", "Load Success\nName: "+name+"\nAlliance: "+DriverStation.getAlliance().toString()+"\nMirrored: "+(flipSide ? "Yes" : "No"));
			SmartDashboard.putBoolean("Auto Loaded", true);
		}catch(IOException e){
			SmartDashboard.putString("Auto Alert", "Load Failure\nAutoCompiler experienced an IO Exception and could not recover.");
			SmartDashboard.putBoolean("Auto Loaded", false);
		}catch(ParseException e){
			SmartDashboard.putString("Auto Alert", "Load Failure\nAutoCompiler experienced an Parse Exception and could not recover.");
			SmartDashboard.putBoolean("Auto Loaded", false);
		}
		if(p==null) return;
		if(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red)
			p=p.flipPath();
		if(flipSide)
			p=p.mirrorPath();
		auto = AutoBuilder.followPath(p);
	}

	public static Command getAuto(){ return auto; }
	public static Command GetAutoBasic(String id){ return AutoBuilder.buildAuto(id); }
}
