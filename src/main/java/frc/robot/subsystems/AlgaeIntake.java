package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.wrappers.MonitoredPIDController;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;

public class AlgaeIntake 
{
    private SparkMax backAlgaeMotor;
    private SparkMaxConfig backAlgaeMotorConfig;
    private SparkMax frontAlgaeMotor;
    private SparkMaxConfig frontAlgaeMotorConfig;
    private static final double PUSH_OUT = 0.0;
    private static final double BRAKE = 0.0;
    private static final double PULL_IN = 0.0;
    private static final double HOLD = 0.0;
    public AlgaeIntake()
    {
        backAlgaeMotor = new SparkMax(28, MotorType.kBrushless);
        backAlgaeMotorConfig = new SparkMaxConfig();

        backAlgaeMotorConfig.inverted(false).idleMode(IdleMode.kBrake);  // Sets direction, as well as what to do when idle.

        backAlgaeMotor.configure(backAlgaeMotorConfig, ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);

        frontAlgaeMotor = new SparkMax(29, MotorType.kBrushless);
        frontAlgaeMotorConfig = new SparkMaxConfig();

        frontAlgaeMotorConfig.follow(backAlgaeMotor.getDeviceId(), false);
        frontAlgaeMotor.configure(frontAlgaeMotorConfig, ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    }
    
    public InstantCommand pullAlgaeIn(String mode) {
        if(mode.equals("algae"))
        return new InstantCommand(() -> backAlgaeMotor.set(PULL_IN));
        else
        return new InstantCommand();
    }

    public InstantCommand pushAlgaeOut(String mode) {
        if(mode.equals("algae"))
        return new InstantCommand(() -> backAlgaeMotor.set(PUSH_OUT));
        else
        return new InstantCommand();
    }

    public InstantCommand hold() {
        return new InstantCommand(() -> backAlgaeMotor.set(HOLD));//# Starting At 5% Power
    }

    public InstantCommand stopIntake() {
        return new InstantCommand(() -> backAlgaeMotor.set(BRAKE));//# Starting At 5% Power
    }
}
