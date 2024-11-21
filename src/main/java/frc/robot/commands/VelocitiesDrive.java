package frc.robot.commands;

import java.util.List;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrivetrain;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

public class VelocitiesDrive extends Command {
    
  private final SwerveDrivetrain swerve;
  private final DoubleSupplier  vX, vY;
  private final DoubleSupplier omega;
  private boolean initRotation = false;

  /**
   * 
   * @param swerveDrive Drivetrain subsystem
   * @param vXSupplier X Speed Supplier m/s
   * @param vYSupplier Y Speed Supplier m/s
   * @param omegaSupplier Rotation Speed Supplier rad/s
   */
  public VelocitiesDrive(SwerveDrivetrain swerveDrive, DoubleSupplier vXSupplier, DoubleSupplier vYSupplier, DoubleSupplier omegaSupplier) {
    this.swerve = swerveDrive;
    this.vX = vXSupplier;
    this.vY = vYSupplier;
    double speed = 2.5;
    this.omega = () -> omegaSupplier.getAsDouble() * speed;

    addRequirements(swerveDrive);
  }
  @Override
  public void initialize()
  {
    initRotation = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    System.out.println(omega.getAsDouble());
    // System.out.println(vX.getAsDouble());
    // System.out.println(vY.getAsDouble());

    // Get the desired chassis speeds based on a 2 joystick module.
    double newAngle = swerve.getHeading().getRadians() + omega.getAsDouble(); //radians
    ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(vX.getAsDouble(), vY.getAsDouble(), Rotation2d.fromRadians(0));
    // ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(vX.getAsDouble(), vY.getAsDouble(), Rotation2d.fromRadians(newAngle));

    // Prevent Movement After Auto
    if (initRotation)
    {
      if (omega.getAsDouble() == 0)
      {
        // Get the curretHeading
        Rotation2d firstLoopHeading = swerve.getHeading();

        // Set the Current Heading to the desired Heading
        desiredSpeeds = swerve.getTargetSpeeds(0, 0, firstLoopHeading.getSin(), firstLoopHeading.getCos());
      }
      //Dont Init Rotation Again
      initRotation = false;
    }

    // Limit velocity to prevent tippy
    Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
    translation = SwerveMath.limitVelocity(translation, swerve.getFieldVelocity(), swerve.getPose(),
                                           Constants.LOOP_TIME, Constants.ROBOT_MASS, List.of(Constants.CHASSIS),
                                           swerve.getSwerveDriveConfiguration());
    SmartDashboard.putNumber("LimitedTranslation", translation.getX());
    SmartDashboard.putString("Translation", translation.toString());

    // Make the robot move
    // System.out.println(translation);
    swerve.drive(translation, omega.getAsDouble(), true);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return false;
  }

}
