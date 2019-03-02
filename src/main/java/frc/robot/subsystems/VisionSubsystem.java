/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class VisionSubsystem extends PIDSubsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  double P;
  double I;
  double D;

  public VisionSubsystem(double P, double I, double D) {
    super(P, I, D);
    this.P = P;
    this.I = I;
    this.D = D;
    setAbsoluteTolerance(5);
    setInputRange(-100, 100);
    setOutputRange(-1, 1);
    setSetpoint(0);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void turnRobot(int speed) {
      
  }

  @Override
  protected double returnPIDInput() {
    return Robot.distance.getDouble(0);
  }

  @Override
  protected void usePIDOutput(double output) {
      Robot.drivesub.turnRobot(output, Robot.oi.joyLeft.getY());
  }

}
