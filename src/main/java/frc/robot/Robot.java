// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cscore.VideoCamera;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private final int RED = 1, BLUE = 0;//May need to reverse these numbers at competition.
  public Robot() 
  {
    m_robotContainer = new RobotContainer(BLUE);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
    SmartDashboard.putNumber("Encoder Position", Constants.elevator.getElevatorPosition());
    SmartDashboard.putNumber("Encoder Velocity:", Constants.elevator.getTargetElevatorSpeed());
    SmartDashboard.putNumber("Desired Height:", Constants.elevator.getDesiredHeight());
    if(Constants.elevator.getElevatorPosition()>240)
    {
      Constants.elevator.emergencyStop();
    }
    SmartDashboard.putNumber("Flipper Angle", Constants.flipper.getFlipperAngle());
    SmartDashboard.putNumber("Flipper Velocity:", Constants.flipper.getTargetFlipperSpeed());
    SmartDashboard.putNumber("Desired Angle:", Constants.flipper.getDesiredAngle());
    if(Constants.flipper.getFlipperAngle()>15)
    {
      Constants.flipper.emergencyStop();
    }

  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) 
    {
      m_autonomousCommand.cancel();
    }
    SmartDashboard.putString("Elevator Mode", "coral");
    SmartDashboard.putString("Speed", "normal");
  }

  @Override
  public void teleopPeriodic()
  {
    Constants.elevator.setElevatorSpeed();
    //Constants.flipper.setFlipperSpeed();
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
