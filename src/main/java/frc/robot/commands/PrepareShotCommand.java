package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.wcp.Hood;
import frc.robot.subsystems.wcp.Landmarks;
import frc.robot.subsystems.wcp.Shooter;

public class PrepareShotCommand extends Command {
    private static final InterpolatingTreeMap<Distance, Shot> distanceToShotMap = new InterpolatingTreeMap<>(
        (startValue, endValue, q) -> 
            InverseInterpolator.forDouble()
                .inverseInterpolate(startValue.in(Meters), endValue.in(Meters), q.in(Meters)),
        (startValue, endValue, t) ->
            new Shot(
                Interpolator.forDouble()
                    .interpolate(startValue.shooterRPM, endValue.shooterRPM, t),
                Interpolator.forDouble()
                    .interpolate(startValue.hoodPosition, endValue.hoodPosition, t)
            )
    );

    private static final double RPM_ADJUSTMENT = 95;
    public static final double STATIC_HOOD_POSITION = .19;

    static {
        distanceToShotMap.put(Inches.of(52.0), new Shot(2800, 0.19));
        distanceToShotMap.put(Inches.of(114.4), new Shot(3275, 0.40));
        distanceToShotMap.put(Inches.of(165.5), new Shot(3650, 0.48));
    }

    private final Shooter shooter;
    private final Hood hood;
    private final Supplier<Pose2d> robotPoseSupplier;

    public PrepareShotCommand(Shooter shooter, Hood hood, Supplier<Pose2d> robotPoseSupplier) {
        this.shooter = shooter;
        this.hood = hood;
        this.robotPoseSupplier = robotPoseSupplier;
        addRequirements(shooter, hood);
    }

    public static Command defaultAim(Hood hood) {
        return Commands.runOnce(() -> hood.setPosition(STATIC_HOOD_POSITION));
    }

    public static Command aimWithDistanceToHub(Shooter shooter, Hood hood, Supplier<Pose2d> robotPoseSupplier) {
        return Commands.runOnce(
            () -> {execute(shooter, hood, robotPoseSupplier);}
        );
    }
    public static Command aimTargetStaticCommand(Shooter shooter, Hood hood, Supplier<Pose2d> robotPoseSupplier) {
        return Commands.runOnce(
            () -> {shooter.setRPM(2800); hood.setPosition(.19);}
        );
    }
    public static Command aimDownrangeCommand(Shooter shooter, Hood hood, Supplier<Pose2d> robotPoseSupplier) {
        return Commands.runOnce(
            () -> {shooter.setRPM(3000); hood.setPosition(1);}
        );
    }
    public static Command haltCommand(Shooter shooter, Hood hood, Supplier<Pose2d> robotPoseSupplier) {
        return Commands.runOnce(
            () -> {shooter.setRPM(0);}
        );
    }

    public boolean isReadyToShoot() {
        return shooter.isVelocityWithinTolerance() && hood.isPositionWithinTolerance();
    }

    private static Distance getDistanceToHub(Supplier<Pose2d> robotPoseSupplier) {
        final Translation2d robotPosition = robotPoseSupplier.get().getTranslation();
        final Translation2d hubPosition = Landmarks.hubPosition();
        return Meters.of(robotPosition.getDistance(hubPosition));
    }
    
    @Override
    public void execute() {
        execute(shooter, hood, robotPoseSupplier);
    }

    public static void execute(Shooter shooter, Hood hood, Supplier<Pose2d> robotPoseSupplier) {
        final Distance distanceToHub = getDistanceToHub(robotPoseSupplier);
        final Shot shot = distanceToShotMap.get(distanceToHub);
        shooter.setRPM(shot.shooterRPM + RPM_ADJUSTMENT);

        // hood.setPosition(shot.hoodPosition);
        hood.setPosition(STATIC_HOOD_POSITION);

        SmartDashboard.putNumber("Distance to Hub (inches)", distanceToHub.in(Inches));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }

    public static class Shot {
        public final double shooterRPM;
        public final double hoodPosition;

        public Shot(double shooterRPM, double hoodPosition) {
            this.shooterRPM = shooterRPM;
            this.hoodPosition = hoodPosition;
        }
    }
}
