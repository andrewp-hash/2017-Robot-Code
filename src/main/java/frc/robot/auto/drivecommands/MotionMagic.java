/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto.drivecommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;;

public class MotionMagic extends CommandBase {
  private final DriveTrainSubsystem m_drive;
  private final Double m_right;
  private final Double m_left;

  public boolean magic_isFinished;

  public MotionMagic(Double right, Double left, DriveTrainSubsystem drive) {
    m_drive = drive;
    m_right = right;
    m_left = left; 

    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.setMotionMagicMode();
    m_drive.resetEncoders();
    magic_isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!magic_isFinished){
      m_drive.elevatorMotionMagic(m_right, m_left);
    }

    if((Math.abs(m_drive.getRightEncoderDistanceMeters()*2) >= Math.abs(m_right)) &&
    (Math.abs(m_drive.getLeftEncoderDistanceMeters()*2) >= Math.abs(m_left))){
      magic_isFinished = true;  
    }
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.configStandardDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return magic_isFinished;
  }
}
