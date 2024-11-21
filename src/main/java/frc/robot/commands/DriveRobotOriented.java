package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrivetrain;
import swervelib.SwerveController;

public class DriveRobotOriented extends Command {

    SwerveDrivetrain swerveDrive;
    DoubleSupplier xSpeed;
    DoubleSupplier ySpeed;
    DoubleSupplier angle;

    //TODO: Change this to headingX and headingY
    public DriveRobotOriented (SwerveDrivetrain swerveDrive, DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier angle) {
        this.swerveDrive = swerveDrive;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.angle = angle;
        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds = swerveDrive.getTargetSpeeds(xSpeed.getAsDouble(), ySpeed.getAsDouble(), Rotation2d.fromRadians(angle.getAsDouble()));
        Translation2d transform = SwerveController.getTranslation2d(speeds);
        transform = transform.rotateBy(Rotation2d.fromRadians(angle.getAsDouble()-Math.PI/2.0));
        //.rotateBy(Rotation2d.fromRadians(angle.getAsDouble()))
        
        swerveDrive.drive(transform, speeds.omegaRadiansPerSecond, true);
    }
}
