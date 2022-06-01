package frc.robot.commands.drives;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.kSwerve;
import frc.robot.subsystems.Drives;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Function;

public class DriveCommand extends CommandBase {
    private final Drives drives;

    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;
    private final DoubleSupplier rotationSupplier;

    private double lastRotationSpeed = 0;
    private Rotation2d setpointAngle = new Rotation2d(); // Degrees
    private BooleanSupplier finished = () -> false;
    private Runnable update = () -> {};
    public DriveCommand(Drives drives, DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier, DoubleSupplier rotationSupplier) {
        this.drives = drives;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        this.rotationSupplier = rotationSupplier;

        addRequirements(drives);
    }
    public DriveCommand(Drives drives, DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier, DoubleSupplier rotationSupplier, BooleanSupplier finished, Runnable update) {
        this.drives = drives;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        this.rotationSupplier = rotationSupplier;
        this.finished = finished;
        this.update = update;

        addRequirements(drives);
    }

    @Override
    public void execute() {
        if(DriverStation.isTeleop() && drives.getRunDrives()) {
            double rotationSpeed = rotationSupplier.getAsDouble();

            // correct for angle 
            if (rotationSpeed == 0) {
                if (lastRotationSpeed != 0) {
                    setpointAngle = drives.getGyroscopeRotation();
                }
                Rotation2d angleOffset = setpointAngle.minus(drives.getGyroscopeRotation());

                if (Math.abs(angleOffset.getDegrees()) > kSwerve.SWERVE_ALLOWED_OFFSET) {
                    rotationSpeed = kSwerve.SWERVE_CORRECTION_SPEED * (angleOffset.getDegrees() / Math.abs(angleOffset.getDegrees()));
                }
            }
            update.run();
            drives.updateModules(
                    drives.getKinematics().toSwerveModuleStates(
                            ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translationXSupplier.getAsDouble(),
                                    translationYSupplier.getAsDouble(),
                                    rotationSpeed,
                                    drives.getGyroscopeRotation()
                            )
                    )
            );

            lastRotationSpeed = rotationSpeed;
        }
    }

    @Override
    public void end(boolean interrupted) {
        drives.updateModules(drives.getKinematics().toSwerveModuleStates(new ChassisSpeeds(0.0, 0.0, 0.0)));
    }
    @Override
    public boolean isFinished(){
        return finished.getAsBoolean();
    }
}
