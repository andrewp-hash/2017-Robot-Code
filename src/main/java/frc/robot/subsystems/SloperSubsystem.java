/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SloperSubsystem extends SubsystemBase {
  WPI_TalonSRX leftSloper = new WPI_TalonSRX(1);
  WPI_TalonSRX rightSloper = new WPI_TalonSRX(2);

  /**
   * Creates a new Sloper.
   */
  public SloperSubsystem() {
    //rightSloper.follow(leftSloper);
  }

  public void sloperUp(){
    leftSloper.set(ControlMode.PercentOutput, 1);
    rightSloper.set(ControlMode.PercentOutput, 1);
  }


  public void sloperDown(){
    leftSloper.set(ControlMode.PercentOutput, -1);
    rightSloper.set(ControlMode.PercentOutput, -1);
  }

  public void sloperStop(){
    leftSloper.set(ControlMode.PercentOutput, 0);
    rightSloper.set(ControlMode.PercentOutput, 0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
