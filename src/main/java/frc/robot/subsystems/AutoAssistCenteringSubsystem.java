/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;


/**
 * This subsystem is here to be used by the commands because the pidsubsystem cannot
 * be initialized in robotInit
 */
public class AutoAssistCenteringSubsystem extends Subsystem 
{
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() 
  {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  public void DriveOverLine()
  {
    Robot.robotmap.flTalon.set(ControlMode.PercentOutput, Robot.lineCentering.getOutput() * .5);
    Robot.robotmap.frTalon.set(ControlMode.PercentOutput, Robot.lineCentering.getOutput() * .5);
    Robot.robotmap.blTalon.set(ControlMode.PercentOutput, Robot.lineCentering.getOutput() * .5);
    Robot.robotmap.brTalon.set(ControlMode.PercentOutput, Robot.lineCentering.getOutput() * .5);
  }
}