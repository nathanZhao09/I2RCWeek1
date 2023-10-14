// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class PidTurnCCW extends CommandBase {
    DriveTrain dt;
    double angle;
    double p=0.3/90;//proportinal consant <1 motorpower/setpoint
    double i=0;
    double d;
    int MotorSign;  //if angle is >1 then 
    PIDController pid= new PIDController(p,i,d);

  /** Creates a new PidTurnCCW. */
  public PidTurnCCW(DriveTrain dt, double angle) {
    this.dt=dt;
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
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
