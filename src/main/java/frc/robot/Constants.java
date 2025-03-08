package frc.robot;


import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Flipper;
import frc.robot.subsystems.Climber;

import static edu.wpi.first.units.Units.*;

public final class Constants //Constants that must be accessed in multiple classes.
{
    public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    public static final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    public static final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    public static final Elevator elevator = new Elevator();

    public static final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public static final CoralIntake coralIntake = new CoralIntake();

    public static final Flipper flipper = new Flipper();

    public static final VideoCamera m_videoCamera = new VideoCamera();

    public static final AlgaeIntake algaeIntake = new AlgaeIntake();

    public static final Climber climber = new Climber();

    private static String mode = "coral";

    public static String getMode()
    {
        return mode;
    }

    public static InstantCommand switchModes()
    {
        if(mode.equals("coral"))
        {
            SmartDashboard.putString("Elevator Mode", "algae");
            return new InstantCommand(() -> mode = "algae");
        }
        else
        {
            SmartDashboard.putString("Elevator Mode", "coral");
            return new InstantCommand(() -> mode = "coral");
        }
    }
    private static double speedFactor = 0.25;//Between -1 and 1
    public static double getSpeedFactor()
    {
        return speedFactor;
    }
    public static InstantCommand switchSpeeds()
    {
        if(speedFactor==0.20)
        {
            SmartDashboard.putString("Speed", "Turbo");
            return new InstantCommand(() -> speedFactor = 0.5);//Set to 1 if necessary.
        }
        else
        {
            SmartDashboard.putString("Speed", "Normal");
            return new InstantCommand(() -> speedFactor = 0.25);
        }
    }


}