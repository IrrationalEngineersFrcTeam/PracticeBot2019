/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.robot.subsystems.AutoAssistCenteringSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionLineCentering;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionTargetCentering;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static OI oi;
  public static RobotMap robotmap;
  public static DriveSubsystem drivesub;
  public static VisionSubsystem visionSub;
  public static AutoAssistCenteringSubsystem autocenteringsub;
  public static VisionLineCentering lineCentering;
  public static VisionTargetCentering targetCentering;
  public static NetworkTableInstance inst;
  public static NetworkTable smartDashboardTable;
  public static NetworkTable camera1Table;
  public static NetworkTable camera2Table;
  public static NetworkTableEntry connected;
  public static NetworkTableEntry timeRunning;
  public static NetworkTableEntry distance;
  public static NetworkTableEntry LineCenteringDistance;
  public static NetworkTableEntry VisionTargetDist;
  public static NetworkTableEntry piTest;
  public static NetworkTableEntry encoderL;
  public static NetworkTableEntry encoderR;
  public static NetworkTableEntry NavXYaw;
  public static NetworkTableEntry xDiff;
  public static NetworkTableEntry yDiff;
  public static NetworkTableEntry VisionTargetIsSeen;

  public static double PIDTurn;
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    robotmap = new RobotMap();
    drivesub = new DriveSubsystem();
    visionSub = new VisionSubsystem(1, 0, 0);
    inst = NetworkTableInstance.getDefault();
    lineCentering = new VisionLineCentering(1.0, 0.0, 0.0);
    smartDashboardTable = inst.getTable("SmartDashboard");
    camera1Table = inst.getTable("Camera1");
    camera2Table = inst.getTable("Camera2");
    connected = smartDashboardTable.getEntry("robotConnection");
    LineCenteringDistance = camera2Table.getEntry("yDiff");
    timeRunning = smartDashboardTable.getEntry("timeRunning");
    VisionTargetDist = camera1Table.getEntry("distance");
    encoderL = smartDashboardTable.getEntry("encoderL");
    NavXYaw = smartDashboardTable.getEntry("NavXYaw");
    encoderR = smartDashboardTable.getEntry("encoderR");
    yDiff = camera2Table.getEntry("yDiff");
    timeRunning.setBoolean(true);
    VisionTargetIsSeen = camera1Table.getEntry("isSeen");
    targetCentering = new VisionTargetCentering();
    autocenteringsub = new AutoAssistCenteringSubsystem();
    oi = new OI();
    //piTest = smartDashboardTable.getEntry("timeRunning");
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  Joystick joy1 = new Joystick(0);
  Joystick joy2 = new Joystick(1);
 
  @Override
  public void robotPeriodic() {

    //System.out.println(distance.getDouble(0));
    connected.setBoolean(true);

    PIDTurn = VisionTargetDist.getDouble(0);
    //System.out.println(PIDTurn);
    
    //piTest.setDouble(distance.getDouble(0));
    //encoderL.setDouble(leftspeed);
    //encoderR.setDouble(rightspeed);
    //System.out.println(smartDashboardTable.containsKey("key"));

  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This function is called periodically during operator control.
   */

  @Override
  public void teleopInit() {
    //TODO move to autonomous for actual competition
    timeRunning.setBoolean(true);
  }

  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    Scheduler.getInstance().run();

  }
}
