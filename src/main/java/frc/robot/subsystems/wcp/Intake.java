package frc.robot.subsystems.wcp;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
// import static edu.wpi.first.units.Units.RotationsPerSecond;
// import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
// import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
// import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
// import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

// import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
// import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.KrakenX60;
import frc.robot.Constants.Ports;
// import frc.robot.subsystems.stock.SwerveSubsystem;

public class Intake extends SubsystemBase {
	
	private double offset = 0; // custom
	private boolean calibrating = false; // custom
	final AnalogInput limitSwitch = new AnalogInput(Ports.kIntakeLimitSwitch);
	final PIDController pivotPID = new PIDController(0.1, 0, 0);
	private double target = 0;
	
	public enum Speed {
		STOP(0),
		INTAKE(0.8);

		private final double percentOutput;

		private Speed(double percentOutput) {
			this.percentOutput = percentOutput;
		}

		public Voltage voltage() {
			return Volts.of(percentOutput * 12.0);
		}
	}

	public enum Position {
		STOWED(0),
		INTAKE(-122),
		AGITATE(50-122);

		private final double degrees;

		private Position(double degrees) {
			this.degrees = degrees;
		}

		public Angle angle() {
			return Degrees.of(degrees);
		}
	}

	private static final double kPivotReduction = 50.0;
	// private static final AngularVelocity kMaxPivotSpeed = KrakenX60.kFreeSpeed.div(kPivotReduction);
	private static final Angle kPositionTolerance = Degrees.of(5);

	private final TalonFX pivotMotor, rollerMotor;
	// private final VoltageOut pivotVoltageRequest = new VoltageOut(0);
	// private final MotionMagicVoltage pivotMotionMagicRequest = new MotionMagicVoltage(0).withSlot(0);
	private final VoltageOut rollerVoltageRequest = new VoltageOut(0);

	// private boolean isHomed = false;

	public Intake() {
		pivotMotor = new TalonFX(Ports.kIntakePivot, Ports.kCANivoreCANBus);
		rollerMotor = new TalonFX(Ports.kIntakeRollers, Ports.kRoboRioCANBus);
		configurePivotMotor();
		configureRollerMotor();
		SmartDashboard.putData(this);
	}

	private void configurePivotMotor() {
		final TalonFXConfiguration config = new TalonFXConfiguration()
			.withMotorOutput(
				new MotorOutputConfigs()
					.withInverted(InvertedValue.CounterClockwise_Positive)
					.withNeutralMode(NeutralModeValue.Brake)
			)
			.withCurrentLimits(
				new CurrentLimitsConfigs()
					.withStatorCurrentLimit(Amps.of(120))
					.withStatorCurrentLimitEnable(true)
					.withSupplyCurrentLimit(Amps.of(70))
					.withSupplyCurrentLimitEnable(true)
			)
			.withFeedback(
				new FeedbackConfigs()
					.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
					.withSensorToMechanismRatio(kPivotReduction)
			);
		pivotMotor.getConfigurator().apply(config);
	}

	private void configureRollerMotor() {
		final TalonFXConfiguration config = new TalonFXConfiguration()
			.withMotorOutput(
				new MotorOutputConfigs()
					.withInverted(InvertedValue.Clockwise_Positive)
					.withNeutralMode(NeutralModeValue.Brake)
			)
			.withCurrentLimits(
				new CurrentLimitsConfigs()
					.withStatorCurrentLimit(Amps.of(120))
					.withStatorCurrentLimitEnable(true)
					.withSupplyCurrentLimit(Amps.of(70))
					.withSupplyCurrentLimitEnable(true)
			);
		rollerMotor.getConfigurator().apply(config);
	}

	private boolean isPositionWithinTolerance() {
		final Angle currentPosition = pivotMotor.getPosition().getValue();
		// final Angle targetPosition = new pivotMotionMagicRequest.getPositionMeasure();
		return currentPosition.isNear(Degrees.of(target+offset), kPositionTolerance);
	}

	// private void setPivotPercentOutput(double percentOutput) {
	// 	pivotMotor.setControl(
	// 		pivotVoltageRequest
	// 			.withOutput(Volts.of(percentOutput * 12.0))
	// 	);
	// }

	public void set(Position position) {
		target = position.angle().in(Degrees);
	}

	public void set(Speed speed) {
		if(!calibrating) // custom
			rollerMotor.setControl(
				rollerVoltageRequest
					.withOutput(speed.voltage())
			);
		else
			rollerMotor.setControl(
				rollerVoltageRequest
					.withOutput(Speed.STOP.voltage())
			);
	}

	/**
	 * When the command starts, the pivot is moved to the INTAKE position and the
	 * roller motor is set to the INTAKE speed. The command continues running until
	 * it is interrupted or ends.
	 */
	public Command intakeCommand() {
		return runOnce(() -> {
			set(Speed.INTAKE);
			set(Position.INTAKE);
		});
	}

	public Command closeCommand() {
		return runOnce(() -> {
			set(Position.STOWED);
			set(Speed.STOP);
		});
	}

	public Command haltCommand() {
		return runOnce(() -> set(Speed.STOP));
	}

	/**
	 * The command begins by setting the roller motor to the INTAKE speed. It then
	 * continuously cycles the pivot between the AGITATE and INTAKE positions. After
	 * each movement, the command waits until the pivot reaches the target position
	 * within tolerance before continuing. This sequence repeats indefinitely until
	 * the command is interrupted. When the command is interrupted, the pivot is
	 * returned to the INTAKE position and the roller motor is stopped.
	 */
	public Command agitateCommand() {
		return Commands.sequence(
					runOnce(() -> set(Position.AGITATE)),
					Commands.waitUntil(this::isPositionWithinTolerance),
					runOnce(() -> set(Position.INTAKE)),
					Commands.waitUntil(this::isPositionWithinTolerance)
				)
				.repeatedly()
			.handleInterrupt(() -> {
				set(Position.INTAKE);
			});
	}
	
	/**
	 *  Custom Methods Made By DiOrio
	 */
	public boolean isCalibrating(){return calibrating;}
	class Calibrate extends Command {
		private final Intake intake;

		public Calibrate(Intake intake){
			super();
			this.intake=intake;
		}

		@Override public void initialize(){intake.beginCalibrating();}

		@Override public void execute(){}

		@Override public boolean isFinished(){return !intake.isCalibrating();}

		@Override public void end(boolean interrupted){}

	}
	public Command calibrateCommand() { return new Calibrate(this); }
	public void beginCalibrating(){ 
		calibrating=true; 
		// offset=pivotMotor.getPosition().getValue().in(Degrees)*360; 
	}

	@Override
	public void periodic(){
		if(calibrating){
			pivotMotor.setControl(new VoltageOut(1.5));
			if(limitSwitch.getVoltage()<.1){
				calibrating=false;
				offset=pivotMotor.getPosition().getValue().in(Degrees);
			}
		}else{
			pivotMotor.setControl(new VoltageOut(
				Math.min(Math.max(
					pivotPID.calculate(pivotMotor.getPosition().getValue().in(Degrees), target+offset),
					-2),2)
				));
		}
	}

	/*
	 * Initializes the sendable properties for this subsystem to display on the dashboard.
	 * 
	 * <pre>
	 * +----------------------------------+
	 * |        IntakeSubsystem           |
	 * +----------------------------------+
	 * | Command:              IntakeIn   |
	 * |                                  |
	 * | Angle (degrees):      42.5°      |
	 * | RPM:                  1800       |
	 * |                                  |
	 * | Pivot Supply Current:  6.3 A     |
	 * | Roller Supply Current: 4.9 A     |    
	 * +----------------------------------+
	 * </pre>
	 */
	@Override
	public void initSendable(SendableBuilder builder) {
		builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
		builder.addDoubleProperty("Angle (degrees)", () -> pivotMotor.getPosition().getValue().in(Degrees), null);
		builder.addDoubleProperty("RPM", () -> rollerMotor.getVelocity().getValue().in(RPM), null);
		builder.addDoubleProperty("Pivot Supply Current", () -> pivotMotor.getSupplyCurrent().getValue().in(Amps), null);
		builder.addDoubleProperty("Roller Supply Current", () -> rollerMotor.getSupplyCurrent().getValue().in(Amps), null);
		builder.addDoubleProperty("Limit Switch", () -> limitSwitch.getVoltage(), null);
	}
}
