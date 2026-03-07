package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.stock.SwerveSubsystem;

public class AutoCompiler {
	private static SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();
	private static SwerveSubsystem swerve;
	public static void SetUpCompiler(SwerveSubsystem _swerve){
		swerve=_swerve;
		autoChooser.setDefaultOption("Do Nothing", Commands.none());
		autoChooser.addOption("Drive Forward", swerve.driveForward().withTimeout(1));
		SmartDashboard.putData("Auto Chooser", autoChooser);
	}
	public static Command compileWithChooser(){
		return AutoBuilder.buildAuto("Middle Shoot"); //autoChooser.getSelected();
	}
}
