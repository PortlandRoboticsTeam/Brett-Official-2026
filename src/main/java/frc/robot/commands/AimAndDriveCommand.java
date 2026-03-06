package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DriveInputSmoother;
import frc.robot.GeometryUtil;
import frc.robot.Constants.Driving;
import frc.robot.subsystems.stock.SwerveSubsystem;
import frc.robot.subsystems.wcp.Landmarks;

public class AimAndDriveCommand extends Command {
    private static final Angle kAimTolerance = Degrees.of(5);

    private final SwerveSubsystem swerve;
    private final DoubleSupplier forwardInput;
    private final DoubleSupplier leftInput;
    // private final DriveInputSmoother inputSmoother;

    private final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngleRequest = new SwerveRequest.FieldCentricFacingAngle()
        .withRotationalDeadband(Driving.kPIDRotationDeadband)
        .withMaxAbsRotationalRate(Driving.kMaxRotationalRate)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo)
        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
        .withHeadingPID(5, 0, 0);

    public AimAndDriveCommand(
        SwerveSubsystem swerve,
        DoubleSupplier forwardInput,
        DoubleSupplier leftInput
    ) {
        this.swerve = swerve;
        this.forwardInput = forwardInput;
        this.leftInput = leftInput;
        addRequirements(swerve);
    }

    public AimAndDriveCommand(SwerveSubsystem swerve) {
        this(swerve, () -> 0, () -> 0);
    }

    public Rotation2d getOperatorForwardDirection(SwerveSubsystem swerve) {
        return swerve.isRedAlliance() ? Rotation2d.fromDegrees(0) : Rotation2d.fromDegrees(180);
    }

    public boolean isAimed() {
        final Rotation2d targetHeading = fieldCentricFacingAngleRequest.TargetDirection;
        final Rotation2d currentHeadingInBlueAlliancePerspective = swerve.getPose().getRotation();
        final Rotation2d currentHeadingInOperatorPerspective = currentHeadingInBlueAlliancePerspective.rotateBy(
            getOperatorForwardDirection(swerve));
        return GeometryUtil.isNear(targetHeading, currentHeadingInOperatorPerspective, kAimTolerance);
    }

    private Rotation2d getDirectionToHub() {
        final Translation2d hubPosition = Landmarks.hubPosition();
        final Translation2d robotPosition = swerve.getPose().getTranslation();
        final Rotation2d hubDirectionInBlueAlliancePerspective = hubPosition.minus(robotPosition).getAngle();
        final Rotation2d hubDirectionInOperatorPerspective = hubDirectionInBlueAlliancePerspective.rotateBy(
            getOperatorForwardDirection(swerve));
        return hubDirectionInOperatorPerspective;
    }

    @Override
    public void execute() {
        DriveInputSmoother smoother = new DriveInputSmoother(forwardInput, leftInput);
        Vector<N2> smoothedInput = smoother.getSmoothedInput();
        
        swerve.aimAtRequest(
            fieldCentricFacingAngleRequest
                .withVelocityX(Driving.kMaxSpeed.times(smoothedInput.get(0)))
                .withVelocityY(Driving.kMaxSpeed.times(smoothedInput.get(1)))
                .withTargetDirection(getDirectionToHub())
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
