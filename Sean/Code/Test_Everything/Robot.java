/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

  Gamepad gamepad1 = new Gamepad(0);

  /*Zuko: FL:2, RL:3, FR:1, RR:11
    Alorac: L:3 and 1, R:2 and 0 */
  private static final WPI_TalonSRX talonFrontLeft = new WPI_TalonSRX(3);
  private static final WPI_TalonSRX talonRearLeft = new WPI_TalonSRX(1);
  private SpeedControllerGroup driveLeft = new SpeedControllerGroup(talonFrontLeft, talonRearLeft);
  private static final WPI_TalonSRX talonFrontRight = new WPI_TalonSRX(2);
  private static final WPI_TalonSRX talonRearRight = new WPI_TalonSRX(0);
  private SpeedControllerGroup driveRight = new SpeedControllerGroup(talonFrontRight, talonRearRight);

  private DifferentialDrive drive = new DifferentialDrive(driveLeft, driveRight);

  private Timer timer1 = new Timer();
  private double currTime;
  private double prevTime;

  private int i = 0;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    quickThread sensorThread = new quickThread();
    sensorThread.start();
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
    
    talonFrontLeft.configOpenloopRamp(0);
    talonRearLeft.configOpenloopRamp(0);
    talonFrontRight.configOpenloopRamp(0);
    talonRearRight.configOpenloopRamp(0);

    double power = gamepad1.getAxisLeftStickY();
    double rotation = -gamepad1.getAxisRightStickX();
    SmartDashboard.putNumber("leftY", power);
    SmartDashboard.putNumber("RightX", rotation);
    SmartDashboard.putNumber("CurrFL", talonFrontLeft.getOutputCurrent());
    SmartDashboard.putNumber("MotorVoltFL", talonFrontLeft.getMotorOutputVoltage());
    SmartDashboard.putNumber("CurrRL", talonRearLeft.getOutputCurrent());
    SmartDashboard.putNumber("MotorVoltRL", talonRearLeft.getMotorOutputVoltage());
    SmartDashboard.putNumber("CurrFR", talonFrontRight.getOutputCurrent());
    SmartDashboard.putNumber("MotorVoltFR", talonFrontRight.getMotorOutputVoltage());
    SmartDashboard.putNumber("CurrRR", talonRearRight.getOutputCurrent());
    SmartDashboard.putNumber("MotorVoltRR", talonRearRight.getMotorOutputVoltage());
    SmartDashboard.putNumber("i", i);

    currTime = timer1.getFPGATimestamp();
    SmartDashboard.putNumber("looprate", currTime - prevTime);
    prevTime = currTime;
    SmartDashboard.putNumber("looprate_getperiod", getPeriod());

    drive.arcadeDrive(power, rotation);
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  private class quickThread extends Thread
  {
    @Override
    public void run() {
      try
      {
        while (!isInterrupted()) 
        {
          i = i + 1;
          Thread.sleep(5);
        }
      } 
      catch (InterruptedException e) 
      {
        System.out.println(e.getMessage());
      }
      catch (Exception e) 
      {
        System.out.println(e.getMessage());
      }
    }
  }
}
