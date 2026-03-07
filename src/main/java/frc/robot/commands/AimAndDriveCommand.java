package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.FeetPerSecond;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DriveInputSmoother;
import frc.robot.GeometryUtil;
import frc.robot.subsystems.stock.SwerveSubsystem;
import frc.robot.subsystems.wcp.Landmarks;

public class AimAndDriveCommand extends Command {
    private static final Angle kAimTolerance = Degrees.of(0.5);

    private final SwerveSubsystem swerve;
    private final DoubleSupplier forwardInput;
    private final DoubleSupplier leftInput;
    // private final DriveInputSmoother inputSmoother;

    private final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngleRequest = new SwerveRequest.FieldCentricFacingAngle()
        .withRotationalDeadband(0.01)
        .withMaxAbsRotationalRate(Units.degreesToRadians(540))
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo)
        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
        .withHeadingPID(150, 0.6, 3);

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

    public static Rotation2d getOperatorForwardDirection(SwerveSubsystem swerve) {
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
        return getDirectionToHub(swerve);
    }

    public static Rotation2d getDirectionToHub(SwerveSubsystem swerve) {
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
                .withVelocityX(FeetPerSecond.of(14.5).times(smoothedInput.get(0)))
                .withVelocityY(FeetPerSecond.of(14.5).times(smoothedInput.get(1)))
                .withTargetDirection(getDirectionToHub())
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
