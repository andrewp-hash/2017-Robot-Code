/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrainSubsystem extends SubsystemBase {
  // this is the motors and their ports n stuff
  WPI_TalonSRX leftDriveMain = new WPI_TalonSRX(12);
  WPI_TalonSRX leftDriveSecondary = new WPI_TalonSRX(14);
  WPI_TalonSRX rightDriveMain = new WPI_TalonSRX(11);
  WPI_TalonSRX rightDriveSecondary = new WPI_TalonSRX(13);

  // AHRS gyro = new AHRS(SPI.Port.kMXP);
  // NavX gyroscope = new NavX(SPI.Port.kMXP);

  Pose2d pose;

  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(23));
  // DifferentialDriveOdometry odometry = new
  // DifferentialDriveOdometry(getHeading());

  SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(Constants.kS, Constants.kV, Constants.kA);; // need to
                                                                                                              // run
                                                                                                              // diagnostics,
                                                                                                              // 42.25

  PIDController leftPidController = new PIDController(Constants.kP, 0.0, 0.0);
  PIDController rightPidController = new PIDController(Constants.kP, 0.0, 0.0);

  // used to get the sub command m_robotDrive and specify that it is a mecanum
  // drive
  MecanumDrive m_robotDrive = new MecanumDrive(leftDriveMain, leftDriveSecondary, rightDriveMain, rightDriveSecondary);

  DifferentialDrive drive_t = new DifferentialDrive(rightDriveMain, leftDriveMain);

  private boolean m_configCorrectly = true;

  // Creates a new DriveTrain, just here so that it jnows what it is
  public DriveTrainSubsystem() {
  }

  /*this is added to specify the drive type
   use double to get #s and speed
   turn strafe are the three directions
   invert if driving the wrong dirrection*/
  public void MecanumDrive(double speed, double turn, double strafe){
    m_robotDrive.driveCartesian(-speed, turn, -strafe);
  }

  public void setMaxOutput(double maxOutput){
  m_robotDrive.setMaxOutput(1);
  }

    
/*************************diffrent systems to set up motion magic/auto***********************/
  //configs everything
  public void setMotionMagicMode(){
 
  //configs the defualt
    leftDriveMain.configFactoryDefault();
    leftDriveSecondary.configFactoryDefault();
    rightDriveMain.configFactoryDefault();
    rightDriveSecondary.configFactoryDefault();
    
    //Configures the MagEncoders into Relative mode
    leftDriveMain.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor); //figure out how to put in sensors and thier code
    rightDriveMain.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    leftDriveMain.setSelectedSensorPosition(0);
    rightDriveMain.setSelectedSensorPosition(0);

    //Set the sensor phase
    leftDriveMain.setSensorPhase(false);
    rightDriveMain.setSensorPhase(true);

    //set the saftey enabled to false
    drive_t.setSafetyEnabled(false);

    //This makes the slave controllers follow the output values of the master controllers
    leftDriveSecondary.follow(leftDriveMain);
    rightDriveSecondary.follow(rightDriveMain);

    //configs the brake mode
    leftDriveMain.setNeutralMode(NeutralMode.Coast);
    rightDriveMain.setNeutralMode(NeutralMode.Coast);
    leftDriveSecondary.setNeutralMode(NeutralMode.Coast);
    rightDriveSecondary.setNeutralMode(NeutralMode.Coast);


    m_configCorrectly = false;
}



public void configStandardDrive(){
  leftDriveMain.configFactoryDefault();
  leftDriveSecondary.configFactoryDefault();
  rightDriveSecondary.configFactoryDefault();
  rightDriveMain.configFactoryDefault();
  
  //Configures the MagEncoders into Relative mode
  leftDriveMain.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
  rightDriveMain.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
  leftDriveMain.setSelectedSensorPosition(0);
  rightDriveMain.setSelectedSensorPosition(0);

  //Live life on the edge and turn off safety mode
  m_robotDrive.setSafetyEnabled(false);

  //Set the sensor phase
  leftDriveMain.setSensorPhase(true);
  rightDriveMain.setSensorPhase(true);

  leftDriveMain.setNeutralMode(NeutralMode.Coast);
  rightDriveMain.setNeutralMode(NeutralMode.Coast);
  leftDriveSecondary.setNeutralMode(NeutralMode.Coast);
  rightDriveSecondary.setNeutralMode(NeutralMode.Coast);
}

/***********************************extra features i want***********************************/
 
//start the e brake
  public void ebrake(){
    leftDriveMain.setNeutralMode(NeutralMode.Brake);
    rightDriveMain.setNeutralMode(NeutralMode.Brake);
    leftDriveSecondary.setNeutralMode(NeutralMode.Brake);
    rightDriveSecondary.setNeutralMode(NeutralMode.Brake);
  }

  //end the e brake
  public void no_ebrake(){
    leftDriveMain.setNeutralMode(NeutralMode.Coast);
    rightDriveMain.setNeutralMode(NeutralMode.Coast);
    leftDriveSecondary.setNeutralMode(NeutralMode.Coast);
    rightDriveSecondary.setNeutralMode(NeutralMode.Coast);
  }


/**********************************robot gyro and encoder stuff****************************/

 /*
  public double getYaw(){
    return -gyro.getAngle();
  }
  */

  /*Gets the angle of the robot
  public Rotation2d getHeading(){
    return Rotation2d.fromDegrees(-gyro.getAngle());
  }
  */

  public double getLeftEncoderDistanceMeters() {
    double leftDistance;
    leftDistance = leftDriveMain.getSelectedSensorPosition() / Constants.encoderTicksPerRev * Units.inchesToMeters(Constants.wheelCircumferenceInches);
    return leftDistance;
  }

  public double getRightEncoderDistanceMeters(){
  double rightDistance;
  rightDistance = rightDriveMain.getSelectedSensorPosition() / Constants.encoderTicksPerRev * Units.inchesToMeters(Constants.wheelCircumferenceInches);
  return rightDistance;
  }

  public SimpleMotorFeedforward getFeedforward(){
  return feedForward;
  }

  public PIDController getLeftPIDController(){
  return leftPidController;
  }

  public PIDController getRightPIDController(){
  return rightPidController;
  }

  public void resetEncoders() {
    leftDriveMain.setSelectedSensorPosition(0);
    rightDriveMain.setSelectedSensorPosition(0);
  }

  public void resetHeading() {
    //gyro.reset();
  }

  public void elevatorMotionMagic(double wheelrotRight, double wheelrotLeft){
    if(m_configCorrectly){
      setMotionMagicMode();
    }

    double targetPosition =  Constants.encoderTicksPerRev * wheelrotRight;
    double targetPosition2 = Constants.encoderTicksPerRev * wheelrotLeft;
  
    rightDriveMain.set(ControlMode.MotionMagic, targetPosition);
    leftDriveMain.set(ControlMode.MotionMagic, targetPosition2);
  }


  //blank code that is just needed
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
   // pose = odometry.update(getHeading(), getLeftEncoderDistanceMeters(), getRightEncoderDistanceMeters());

    SmartDashboard.putNumber("left encoder pos", leftDriveMain.getSelectedSensorPosition());
    SmartDashboard.putNumber("right encoder pos", rightDriveMain.getSelectedSensorPosition());

    //SmartDashboard.putNumber("heading", getYaw());

    SmartDashboard.putNumber("left encoder vel", leftDriveMain.getSelectedSensorVelocity());
    SmartDashboard.putNumber("right encoder vel", rightDriveMain.getSelectedSensorVelocity());

   // SmartDashboard.putNumber("X pose", odometry.getPoseMeters().getTranslation().getX());
   // SmartDashboard.putNumber("Y pose", odometry.getPoseMeters().getTranslation().getY());
  }
}
