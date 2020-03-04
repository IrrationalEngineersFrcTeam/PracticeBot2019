/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.subsystems.VisionTargetCentering;
import com.ctre.phoenix.motorcontrol.ControlMode;


/**
 * This subsystem is here to be used by the commands because the pidsubsystem cannot
 * be initialized in robotInit
 */
public class AutoAssistCenteringSubsystem extends Subsystem 
{
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  static double Output = Robot.targetCentering.getOutput();
  // static boolean IsSeen = Robot.VisionTargetIsSeen.getBoolean(true);

  public void FindTarget()
  {

    Robot.robotmap.flTalon.set(ControlMode.PercentOutput, -0.3);
    Robot.robotmap.blTalon.set(ControlMode.PercentOutput, -0.3);
    Robot.robotmap.frTalon.set(ControlMode.PercentOutput, 0.3);
    Robot.robotmap.brTalon.set(ControlMode.PercentOutput, 0.3);

  }

  public void CenterOnTarget()
  {

    //System.out.println(Output);

    if(Output > 0)
    {
      Robot.robotmap.flTalon.set(ControlMode.PercentOutput, Output * 0.5);
      Robot.robotmap.blTalon.set(ControlMode.PercentOutput, Output * 0.5);
    }
    else if(Output < 0)
    {
      Robot.robotmap.frTalon.set(ControlMode.PercentOutput, Output * 0.5);
      Robot.robotmap.brTalon.set(ControlMode.PercentOutput, Output * 0.5);
    }

  }

  @Override
  public void initDefaultCommand() 
  {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void DriveOverLine()
  {
    Robot.robotmap.flTalon.set(ControlMode.PercentOutput, Robot.lineCentering.getOutput() * .5);
    Robot.robotmap.blTalon.set(ControlMode.PercentOutput, Robot.lineCentering.getOutput() * .5);
    //Robot.robotmap.frTalon.set(ControlMode.PercentOutput, Robot.lineCentering.getOutput() * .5);
    //Robot.robotmap.brTalon.set(ControlMode.PercentOutput, Robot.lineCentering.getOutput() * .5);
  }

  public void Stop()
  {
    Robot.robotmap.flTalon.set(ControlMode.PercentOutput, 0.0);
    Robot.robotmap.frTalon.set(ControlMode.PercentOutput, 0.0);
    Robot.robotmap.blTalon.set(ControlMode.PercentOutput, 0.0);
    Robot.robotmap.brTalon.set(ControlMode.PercentOutput, 0.0);
  }
}
