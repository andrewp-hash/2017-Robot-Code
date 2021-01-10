/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.PlayerDrive;
import frc.robot.commands.SloperDownCommand;
import frc.robot.commands.SloperUpCommand;
import frc.robot.commands.SpinnerInCommand;
import frc.robot.commands.SpinnerOutCommand;
import frc.robot.auto.autocommands.SimpleAuto1;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.SloperSubsystem;
import frc.robot.subsystems.SloperSpinnerSubsystem;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /* The robot's subsystems and commands are defined here...
  this here is used to first specify which subsytems are used
  then used to specify the type of controller/joystick we are using*/
  private final DriveTrainSubsystem driveSubsystem = new DriveTrainSubsystem();
  private final DriveTrainSubsystem m_driveTrainSubsystem = new DriveTrainSubsystem();
  private final SloperSubsystem m_sloperSubsystem = new SloperSubsystem();
  private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  private final SloperSpinnerSubsystem m_sloperSpinnerSubsystem = new SloperSpinnerSubsystem();
  public static final XboxController driver = new XboxController(0);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    /* Configure the button bindings
    this here is used to mesure the imputs for speed,turn,and strafe, leftX,leftY,andrightX respectively
    refers to left and right joysticks controlled by repective hands, x and y refers to the dirrection pushed*/
    configureButtonBindings();
    driveSubsystem.setDefaultCommand(new PlayerDrive(
      ()-> driver.getX(Hand.kLeft),
      ()-> driver.getY(Hand.kLeft),
      ()-> -driver.getX(Hand.kRight),
      driveSubsystem
    ));

  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(driver, Button.kBumperRight.value)
    .whenPressed(()-> m_driveTrainSubsystem.ebrake())
    .whenReleased(()-> m_driveTrainSubsystem.no_ebrake());

    /*new JoystickButton(driver, Button.kY.value)
    .whenPressed(()-> m_driveTrain.setMaxOutput(.1))
    .whenReleased(()-> m_driveTrain.setMaxOutput(1)); */

    new JoystickButton(driver, Button.kA.value).whenHeld(new SloperUpCommand(m_sloperSubsystem));

    new JoystickButton(driver, Button.kB.value).whenHeld(new SloperDownCommand(m_sloperSubsystem));

    new JoystickButton(driver, Button.kBumperLeft.value).whenHeld(new ClimberCommand(m_climberSubsystem));

    new JoystickButton(driver, Button.kX.value).whenHeld(new SpinnerInCommand(m_sloperSpinnerSubsystem));

    new JoystickButton(driver, Button.kY.value).whenHeld(new SpinnerOutCommand(m_sloperSpinnerSubsystem));
  }


  public Command getAutonomousCommand() {
      return new SimpleAuto1(driveSubsystem);
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  
}
