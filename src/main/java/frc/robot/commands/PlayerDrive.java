/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;

public class PlayerDrive extends CommandBase {
  /*used to specify the diffrent terms and actualy bring them into the command*/ 
  private final DriveTrainSubsystem m_driveTrain;
  private final DoubleSupplier m_speed;
  private final DoubleSupplier m_turn;
  private final DoubleSupplier m_strafe;

  /**
   * Creates a new PlayerDrive.
   */
  public PlayerDrive(DoubleSupplier speed, DoubleSupplier turn, DoubleSupplier strafe, DriveTrainSubsystem dt) {
    /* Use addRequirements() here to declare subsystem dependencies 
    specefies the diffrent terms used*/
    m_driveTrain = dt;
    m_speed = speed;
    m_turn = turn; 
    m_strafe = strafe;
    addRequirements(m_driveTrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  /* Called every time the scheduler runs while the command is scheduled
  this is where u put the actual drive command which is mecanum in this case
  speicfication in subsystem is need to insert this in here*/
  @Override
  public void execute() {
    m_driveTrain.MecanumDrive(-m_speed.getAsDouble(), -m_turn.getAsDouble(), m_strafe.getAsDouble());
  }

  /* Called once the command ends or is interrupted
  here so that it doesnt keep on driving if intirupted*/
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.MecanumDrive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
