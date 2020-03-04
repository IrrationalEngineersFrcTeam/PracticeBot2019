/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

//This is the PID class from the Camera1Code-Kane translated to java
public class PID 
{

    private double P;
    private double I;
    private double D;

    private double setpoint;
    private double previous_error;
    private double integral;
    


    public PID (double P, double I, double D)
    {

        this.P = P;
        this.I = I;
        this.D = D;

        this.setpoint = 0;
        this.previous_error = 0;
        this.integral = 0;
  
    }

    void SetPoint (double Setpoint)
    {

        this.setpoint = Setpoint;

    }

     public double CalculateSpeed (double input)
     {

        double error = this.setpoint - input;
        this.integral += (error * 0.0333);
        double derivitave = error - this.previous_error;
        this.previous_error = error;
        double speed = this.P*error + this.I*this.integral + this.D*derivitave;
        return speed;
    
    }

}
