// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveTrainPorts;
import frc.robot.subsystems.DriveTrain;

public class PidTurnCCW extends CommandBase {
    DriveTrain dt;
    Constants ct;
    double angle;
    double i=0.003;
    double d;
    int MotorSign;  //if angle is >1 then 
    PIDController pid= new PIDController(DriveTrainPorts.kP,i,d);

  /** Creates a new PidTurnCCW. */
  public PidTurnCCW(DriveTrain dt, double angle) {
    this.dt=dt;
    pid.setTolerance(DriveTrainPorts.kST);
    this.angle=angle;
    addRequirements(dt);
    if (angle>0){//conterclockwise turn if >0
      MotorSign=1;
    }
    else{
      MotorSign=-1; //clockwise turn
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dt.resetNavx();
    dt.tankDrive(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = pid.calculate(dt.getAngle(), angle);
    dt.tankDrive(-output*MotorSign,output*MotorSign);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.tankDrive(0.0,0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
  }
}
