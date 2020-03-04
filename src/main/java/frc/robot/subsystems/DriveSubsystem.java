/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.commands.DriveCommand;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class DriveSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new DriveCommand());
  }

    //this takes the X angle from the limelight and turns it into a motor output
    public double VisionTurn(double AngleX)
    {
  
      double angleX = AngleX/29;
      
      if(Math.abs(AngleX)<=0.2)
      {
      return 0;
      }
      else
      {
      return -angleX * 0.01;
      }
  
    }
  
    //This takes the Y angle from the limelight(in degrees), the angle of the mounting of the limelight(in degrees), and the hight of the limeligh(in inches)
    //and then returns the current distance of the robot to the vision target(in inches)
    public double CurrentRoboDistance(double VisionDegreesY, double MountingDegreesY, double MountingHight)
    {
  
      double VisRadiansY = Math.toRadians(VisionDegreesY);
      double MountRadiansY = Math.toRadians(MountingDegreesY);
  
      //The 98.25 is the hight of the competition target
      double Hight = 98.25 - MountingHight;
      double TangentAngle = Math.tan(VisRadiansY + MountRadiansY);
  
      double CurrentDistance = Hight/TangentAngle;
  
      return CurrentDistance;
  
    }
  
    public double AdjustRoboDistance(double TargetDist, double CurrentDist)
    {
  
      double DeltaX = TargetDist - CurrentDist;
  
      return DeltaX * -0.1;
  
    }

  public void tank() {
    double leftspeed = -Robot.oi.joyLeft.getY() * .6;
    double rightspeed = Robot.oi.joyRight.getY() * .6;

    Robot.encoderL.setDouble(Robot.oi.joyLeft.getY());
    Robot.encoderR.setDouble(Robot.oi.joyRight.getY());
    Robot.NavXYaw.setDouble(Robot.oi.joyRight.getX());
    //System.out.println("yDiff = "+ Robot.yDiff.getDouble(0));

   // System.out.println("LeftSpeed: " + leftspeed + ", RightSpeed: " + rightspeed);

    Robot.robotmap.flTalon.set(ControlMode.PercentOutput, leftspeed);
    Robot.robotmap.frTalon.set(ControlMode.PercentOutput, rightspeed);
    Robot.robotmap.blTalon.set(ControlMode.PercentOutput, leftspeed);
    Robot.robotmap.brTalon.set(ControlMode.PercentOutput, rightspeed);
  }

  public void turnRobot(double turnSpeed, double speed) {

  }

  public void ArcaceDrive()
  {
    //System.out.println("ArcadeDrive");
    double DriveForward = Robot.oi.joyRight.getY();
    //double DriveTurn = Robot.oi.joyRight.getX();
    double DriveTurn = Robot.PIDTurn;
    System.out.println(DriveTurn);

    double LeftSide;
    double RightSide;

    if(DriveForward > 0)
    {
    LeftSide = DriveForward + DriveTurn;
    RightSide = DriveForward - DriveTurn;
    }
    else
    {
      LeftSide = DriveForward - DriveTurn;
      RightSide = DriveForward + DriveTurn;
    }


    Robot.robotmap.flTalon.set(ControlMode.PercentOutput, -LeftSide);
    Robot.robotmap.frTalon.set(ControlMode.PercentOutput, RightSide);
    Robot.robotmap.blTalon.set(ControlMode.PercentOutput, -LeftSide);
    Robot.robotmap.brTalon.set(ControlMode.PercentOutput, RightSide);

  }

}
