/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class FindVisionTargetCommand extends Command {
  public FindVisionTargetCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);

    requires(Robot.autocenteringsub);

  }

  public boolean IsSeen = Robot.VisionTargetIsSeen.getBoolean(true);

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println("Finding the vision target");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    Robot.autocenteringsub.FindTarget();
    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {

    return this.IsSeen;

  }

  // Called once after isFinished returns true
  @Override
  protected void end() {

    System.out.println("Vision Target found!");
    Robot.autocenteringsub.Stop();

  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {

    end();
    
  }
}
