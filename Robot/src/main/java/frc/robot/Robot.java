/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot;


import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;


import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



/****************************************** 
 * Things that work:
 * Meccanum drive
 * pneumatics
 * navX
 * 
 * Things that dont work:
 * Camera?
 * 
 * Things still needed to be implemented:
 * Auto
 * Color sensor
 * Line following
 * skew correction
******************************************/

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends IterativeRobot {

  AHRS ahrs;  
  private MecanumDrive m_robotDrive;
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  

  /************************************ 
   * Analog Varriables
   * 
  ************************************/
AnalogInput AILeft = new AnalogInput(0);
AnalogInput AIRight = new AnalogInput(1);
  int AIScale = (147);
  double AILeftDistance = 0;
  double AILeftVoltage = 0;
  double AIRightDistance = 0;
  double AIRightVoltage = 0;
  double AIRightDistanceRounded = 0;
  double AILeftDistanceRounded = 0;
    /************************************ 
   * Joysticks
   * 
  ************************************/
  Joystick driveStick = new Joystick(0);      
  Joystick operatorStick = new Joystick(1); 
  double joyX = driveStick.getRawAxis(0);
  double joyY = driveStick.getRawAxis(1);
  double joyZ = driveStick.getRawAxis(4);
  /************************************ 
   * pneumatics
   * 
  ************************************/
  Compressor c = new Compressor(0);
  DoubleSolenoid Solenoid1 = new DoubleSolenoid(0, 3);
  DoubleSolenoid Solenoid2 = new DoubleSolenoid(1, 2);
  /************************************ 
   * Talons
   * 
  ************************************/
  WPI_TalonSRX frontLeft = new WPI_TalonSRX(1);
  WPI_TalonSRX frontRight = new WPI_TalonSRX(3);
  WPI_TalonSRX rearRight = new WPI_TalonSRX(4);
  WPI_TalonSRX rearLeft = new WPI_TalonSRX(2);

  double ZRotation = 0;
  int iteration_auto = 0;
  
  
  

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
   /************************************ 
   * Camera Init
   * 
  ************************************/
    UsbCamera camera1 = CameraServer.getInstance().startAutomaticCapture(0);
    camera1.setResolution(640, 480);
    //UsbCamera camera2 = CameraServer.getInstance().startAutomaticCapture(1);
    //camera2.setResolution(640, 480);
    camera1.setFPS(30);
    //camera2.setFPS(30);
    /************************************ 
   * Auto chooser
   * 
  ************************************/
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
  /************************************ 
   * Drive setup
   * 
  ************************************/
    m_robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);
    //ahrs = new AHRS(I2C.Port.kMXP); 
    ahrs = new AHRS(SerialPort.Port.kUSB1);
    
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
    c.setClosedLoopControl(true); //compressor activation
  /************************************ 
   * Ultrasonic Sensor distance calculations
   * 
  ************************************/
  AILeftVoltage = AILeft.getAverageVoltage();
  AILeftDistance = (AILeftVoltage * AIScale);

  AIRightVoltage = AIRight.getAverageVoltage();
  AIRightDistance = (AIRightVoltage * AIScale);
  
  AILeftDistanceRounded = Math.round(AILeftDistance);
  AIRightDistanceRounded = Math.round(AIRightDistance);


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
    m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    iteration_auto = 0;

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    

  
    switch (m_autoSelected) {
      case kCustomAuto:
      iteration_auto ++;
      if(iteration_auto < 250){
        m_robotDrive.driveCartesian(0, 0.5,0);
      }
      else{
        frontLeft.set(0);
        rearLeft.set(0);
        frontRight.set(0);
        rearRight.set(0);
      }
        break;
      case kDefaultAuto:
      default:
        iteration_auto ++;
        if(iteration_auto < 50){
          m_robotDrive.driveCartesian(0, 0.5,0);
        }
        else{
          frontLeft.set(0);
          rearLeft.set(0);
          frontRight.set(0);
          rearRight.set(0);
        }
        break;
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  /************************************ 
   * smart dashboard outputs
   * 
  ************************************/
    SmartDashboard.putNumber("Rounded Left Distance", AILeftDistanceRounded);
    SmartDashboard.putNumber("Rounded Right Distance", AIRightDistanceRounded);
    SmartDashboard.putNumber("Left Distance", AILeftDistance);
    SmartDashboard.putNumber("Right Distance", AIRightDistance);
    SmartDashboard.putNumber("Yaw", ZRotation);
    SmartDashboard.putBoolean("is connected", ahrs.isConnected());
    SmartDashboard.putNumber("XDisp", ahrs.getDisplacementX());
    SmartDashboard.putNumber("YDisp", ahrs.getDisplacementY());
   /************************************ 
   * Robot Drive
   * 
  ************************************/
    m_robotDrive.driveCartesian(driveStick.getRawAxis(0), -driveStick.getRawAxis(1),driveStick.getRawAxis(4));
    if(driveStick.getRawButton(1)){
      ahrs.reset();
    }
  /************************************ 
   * pneumatic solenoids
   * 
  ************************************/
   
  if(operatorStick.getRawButton(1)){
      Solenoid1.set(DoubleSolenoid.Value.kForward);
    }
    else if(operatorStick.getRawButton(2)){
      Solenoid1.set(DoubleSolenoid.Value.kReverse);
    }
    

    if(operatorStick.getRawButton(3)){
      Solenoid2.set(DoubleSolenoid.Value.kForward);
    }
    else if(operatorStick.getRawButton(4)){
      Solenoid2.set(DoubleSolenoid.Value.kReverse);
    }
    Solenoid2.set(DoubleSolenoid.Value.kReverse);

    ZRotation = ahrs.getAngle();
    
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
