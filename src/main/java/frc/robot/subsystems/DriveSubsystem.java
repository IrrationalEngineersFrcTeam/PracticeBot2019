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

  public void tank() {
    double leftspeed = -Robot.oi.joyLeft.getY() * .6;
    double rightspeed = Robot.oi.joyRight.getY() * .6;

    Robot.encoderL.setDouble(Robot.oi.joyLeft.getY());
    Robot.encoderR.setDouble(Robot.oi.joyRight.getY());
    Robot.NavXYaw.setDouble(Robot.oi.joyRight.getX());
    System.out.println("yDiff = "+ Robot.yDiff.getDouble(0));

    System.out.println("LeftSpeed: " + leftspeed + ", RightSpeed: " + rightspeed);

    Robot.robotmap.flTalon.set(ControlMode.PercentOutput, leftspeed);
    Robot.robotmap.frTalon.set(ControlMode.PercentOutput, rightspeed);
    Robot.robotmap.blTalon.set(ControlMode.PercentOutput, leftspeed);
    Robot.robotmap.brTalon.set(ControlMode.PercentOutput, rightspeed);
  }

  public void turnRobot(double turnSpeed, double speed) {

  }

}
