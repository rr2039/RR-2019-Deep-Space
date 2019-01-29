/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.SerialDataType;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.DriverStation;
import com.kauailabs.navx.frc.*;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.TimedRobot;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private static float targetYaw;
  private static final float tolerance = 1;
  private static float yawCorrection = 0;

  private static final int FrontLeftChannel = 1;
  private static final int RearLeftChannel = 2;
  private static final int FrontRightChannel = 3;
  private static final int RearRightChannel = 4;

  private static final WPI_TalonSRX frontLeft = new WPI_TalonSRX(FrontLeftChannel);
  private static final WPI_TalonSRX rearLeft = new WPI_TalonSRX(RearLeftChannel);
  private static final WPI_TalonSRX frontRight = new WPI_TalonSRX(FrontRightChannel);
  private static final WPI_TalonSRX rearRight = new WPI_TalonSRX(RearRightChannel);

  private MecanumDrive m_robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);
  private static AHRS ahrs;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    
    try {
      ahrs = new AHRS(SerialPort.Port.kUSB1);
      ahrs.enableLogging(true);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error in instantiating navX-MXP: " + ex.getMessage(), true);
    } 

    targetYaw = ahrs.getYaw() - 90;
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
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
    m_autoSelected = m_chooser.getSelected();
    // autoSelected = SmartDashboard.getString("Auto Selector",
    // defaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
      // Put default auto code here
        break;
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
      float currentYaw = ahrs.getYaw();
      float error = targetYaw - currentYaw;
      if (Math.abs(error) >= tolerance)
      {
        //This really should be a PID loop
        yawCorrection = -error/1000;
      }
      else {
        //Since the error is within the tolerance do not correct
        yawCorrection = 0;
      }
      m_robotDrive.driveCartesian(0, 0, yawCorrection);
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
