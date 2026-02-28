package frc.robot.subsystems.wcp;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.KrakenX60;
import frc.robot.Constants.Ports;

public class Intake extends SubsystemBase {
    
    private double offset = 0; // custom
    private boolean zeroing = true; // custom
    AnalogInput limitSwitch = new AnalogInput(Ports.kIntakeLimitSwitch);
    
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
        HOMED(110),
        STOWED(100),
        INTAKE(-4),
        AGITATE(20);

        private final double degrees;

        private Position(double degrees) {
            this.degrees = degrees;
        }

        public Angle angle() {
            return Degrees.of(degrees);
        }
    }

    private static final double kPivotReduction = 50.0;
    private static final AngularVelocity kMaxPivotSpeed = KrakenX60.kFreeSpeed.div(kPivotReduction);
    private static final Angle kPositionTolerance = Degrees.of(5);

    private final TalonFX pivotMotor, rollerMotor;
    private final VoltageOut pivotVoltageRequest = new VoltageOut(0);
    private final MotionMagicVoltage pivotMotionMagicRequest = new MotionMagicVoltage(0).withSlot(0);
    private final VoltageOut rollerVoltageRequest = new VoltageOut(0);

    private boolean isHomed = false;

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
            )
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(kMaxPivotSpeed)
                    .withMotionMagicAcceleration(kMaxPivotSpeed.per(Second))
            )
            .withSlot0(
                new Slot0Configs()
                    .withKP(300)
                    .withKI(0)
                    .withKD(0)
                    .withKV(12.0 / kMaxPivotSpeed.in(RotationsPerSecond)) // 12 volts when requesting max RPS
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
        final Angle targetPosition = pivotMotionMagicRequest.getPositionMeasure();
        return currentPosition.isNear(targetPosition, kPositionTolerance);
    }

    private void setPivotPercentOutput(double percentOutput) {
        pivotMotor.setControl(
            pivotVoltageRequest
                .withOutput(Volts.of(percentOutput * 12.0))
        );
    }

    public void set(Position position) {
        if(!zeroing) // custom
            pivotMotor.setControl(
                pivotMotionMagicRequest
                    .withPosition(withOffset(position))
            );
        else
            pivotMotor.setControl(
                pivotMotionMagicRequest
                    .withPosition(withOffset(Position.HOMED))
            );
    }

    public void set(Speed speed) {
        if(!zeroing) // custom
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
        return startEnd(
            () -> {
                set(Position.INTAKE);
                set(Speed.INTAKE);
            },
            () -> set(Speed.STOP)
        );
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
        return runOnce(() -> set(Speed.INTAKE))
            .andThen(
                Commands.sequence(
                    runOnce(() -> set(Position.AGITATE)),
                    Commands.waitUntil(this::isPositionWithinTolerance),
                    runOnce(() -> set(Position.INTAKE)),
                    Commands.waitUntil(this::isPositionWithinTolerance)
                )
                .repeatedly()
            )
            .handleInterrupt(() -> {
                set(Position.INTAKE);
                set(Speed.STOP);
            });
    }
    
    /**
     * The pivot motor is driven slowly forward until the supply current exceeds
     * a threshold, indicating that the mechanism has reached its mechanical home stop.
     * Once this occurs, the pivot encoder position is reset to the HOMED angle, the
     * subsystem is marked as homed, and the pivot is moved to the STOWED position.
     * This command will not run if the mechanism has already been homed.
     *
     * The command uses {@link InterruptionBehavior#kCancelIncoming}, meaning that
     * if another command attempts to interrupt it, the incoming command will be canceled
     * and this command will continue running.
     */
     public Command homingCommand() {
        return Commands.sequence(
            runOnce(() -> setPivotPercentOutput(0.1)),
            Commands.waitUntil(() -> pivotMotor.getSupplyCurrent().getValue().in(Amps) > 6),
            runOnce(() -> {
                pivotMotor.setPosition(Position.HOMED.angle());
                isHomed = true;
                set(Position.STOWED);
            })
        )
        .unless(() -> isHomed)
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }
    
    /**
     *  Custom Methods Made By DiOrio
     */
    private Angle withOffset(Position pos){
        return Angle.ofBaseUnits(pos.angle().in(Degrees)-offset,Degrees);
    }
    public Command calibrateCommand() { return runOnce(()->beginCalibrating()); }
    public void beginCalibrating(){ zeroing=true; offset+=90; }

    @Override
    public void periodic(){
        if(zeroing){
            SmartDashboard.putNumber("Limit Voltage", limitSwitch.getVoltage());
            if(zeroing && limitSwitch.getVoltage()<.1){
                zeroing=false;
                homingCommand().execute();
            }else{
                offset--;
                homingCommand().execute();
            }
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
    }
}
