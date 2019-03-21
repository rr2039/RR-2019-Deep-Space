package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.Quaternion;
import com.kauailabs.navx.frc.AHRS.SerialDataType;
import com.kauailabs.navx.frc.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.drive.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.AxisCamera;
import edu.wpi.first.cameraserver.*;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.AnalogOutput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import javax.lang.model.util.ElementScanner6;


public class Robot extends IterativeRobot 
{
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  boolean level1;
  boolean level2;
  boolean level3;
  boolean bottomliftlimit;
  double rawliftEncoderReading;
  boolean clear;

  double USSLout = 0;
  double USSRout = 0;
  double lineUS;
  boolean lineup;

  double driveXAxis;
  double driveYAxis;
  double driveRotation;
  double liftaxis;

  double operator_ZAxis;
  boolean pneumaticsFire = false;

  boolean cargoEject = false;
  boolean cargoIntake = false;
  
  WPI_TalonSRX liftMotor = new WPI_TalonSRX(14);
  WPI_TalonSRX wristmotor = new WPI_TalonSRX(9);

  WPI_TalonSRX climbL = new WPI_TalonSRX(12);
  WPI_TalonSRX climbR = new WPI_TalonSRX(1);

  WPI_TalonSRX talonFR = new WPI_TalonSRX(0);
  WPI_TalonSRX talonBR = new WPI_TalonSRX(3);

  WPI_TalonSRX talonFL = new WPI_TalonSRX(15);
  WPI_TalonSRX talonBL = new WPI_TalonSRX(13);

  WPI_TalonSRX WclimbL = new WPI_TalonSRX(8);
  WPI_TalonSRX WclimbR = new WPI_TalonSRX(7);

  WPI_VictorSPX leftCargoIntake = new WPI_VictorSPX(6);
  WPI_VictorSPX rightCargoIntake = new WPI_VictorSPX(10);

  Joystick joy1 = new Joystick(0);
  Joystick joy2 = new Joystick(1);

  AnalogInput leftUltrasonic = new AnalogInput(0);
  AnalogInput rightUltrasonic = new AnalogInput(1);

  MecanumDrive mecdrive = new MecanumDrive(talonFL, talonBL, talonFR, talonBR);

  DigitalInput bottomlimit = new DigitalInput(1);

  Compressor compressor = new Compressor(0);
  Solenoid ejectorSolenoid = new Solenoid(6);

  @Override
  public void robotInit() 
  {
    // -----------------------------------------------------------------
    liftMotor.setSensorPhase(true);
    liftMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    liftMotor.config_kP(0, 0.01, 30);
    liftMotor.config_kI(0, 0.25, 30);
    liftMotor.config_kD(0, 0.015, 30);
    liftMotor.config_kF(0, 0.1, 30);
    liftMotor.configClosedLoopPeriod(0, 1, 30);
    liftMotor.configClearPositionOnQuadIdx(clear, 20);

    // -----------------------------------------------------------------
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    // -----------------------------------------------------------------
    compressor.setClosedLoopControl(true);

    // -----------------------------------------------------------------
    // AxisCamera camera2 = CameraServer.getInstance().addAxisCamera("10.20.39.11");
    UsbCamera camera2 = CameraServer.getInstance().startAutomaticCapture(1);
    UsbCamera camera1 = CameraServer.getInstance().startAutomaticCapture(0);
    camera1.setResolution(128, 96);
    camera1.setFPS(30);
    camera2.setResolution(128, 96);
    camera2.setFPS(30);

    // -----------------------------------------------------------------
  }
  @Override
  public void robotPeriodic() 
  {
    // -----------------------------------------------------------------
    level1 = joy2.getRawButton(3);
    level2 = joy2.getRawButton(4);
    level3 = joy2.getRawButton(5);

    // -----------------------------------------------------------------
    USSLout = (int) (leftUltrasonic.getAverageVoltage() * 147);
    USSRout = (int) (rightUltrasonic.getAverageVoltage() * 147);
    SmartDashboard.putNumber("Ultrasonic L", USSLout);
    SmartDashboard.putNumber("Ultrasonic R", USSRout);

    // -----------------------------------------------------------------
    mecdrive.setDeadband(0.05);

    // -----------------------------------------------------------------
    bottomliftlimit = bottomlimit.get();

    // -----------------------------------------------------------------
    rawliftEncoderReading = climbL.getSelectedSensorPosition();
    SmartDashboard.putNumber("Raw Encoder Value", rawliftEncoderReading);

    // -----------------------------------------------------------------
    operator_ZAxis = joy2.getRawAxis(2);
    liftaxis = joy2.getRawAxis(1);

    // -----------------------------------------------------------------
    driveXAxis = joy1.getRawAxis(0);
    driveYAxis = joy1.getRawAxis(1);
    driveRotation = joy1.getRawAxis(4); 
    lineup = joy1.getRawButton(1);
    mecdrive.driveCartesian(driveXAxis , -driveYAxis , driveRotation);

    // -----------------------------------------------------------------
    if (lineup)
    {
      USCorrection();
    }

    // -----------------------------------------------------------------
    if(joy2.getRawButton(1))
    {
      Autolift();
    }

    // -----------------------------------------------------------------    
    if (joy2.getRawButton(6))
    {
      wristmotor.set(ControlMode.PercentOutput, -0.8);
    }
    else if (joy2.getRawButton(7))
    {
      wristmotor.set(ControlMode.PercentOutput, 0.5);
    }
    else
    {
      wristmotor.set(ControlMode.PercentOutput, 0.0);
    }

    // -----------------------------------------------------------------
    if (operator_ZAxis < 0.85 && operator_ZAxis  > -0.85)
    {
      pneumaticsFire = joy2.getRawButton(2);
      cargoIntake = false;
      cargoEject = false;
    }
    else if (operator_ZAxis >= 0.85 )
    {
      cargoEject = joy2.getRawButton(2);
      cargoIntake = false;
      pneumaticsFire = false;
    }
    else if (operator_ZAxis <= -0.85)
    {
      cargoIntake = joy2.getRawButton(2);
      cargoEject = false;
      pneumaticsFire = false;
    }

    // -----------------------------------------------------------------
    if(pneumaticsFire)
    {
      ejectorSolenoid.set(true);
    }
    else
    {
      ejectorSolenoid.set(false);
    }

    // -----------------------------------------------------------------
    if (cargoEject)
    {
      leftCargoIntake.set(-0.45);
      rightCargoIntake.set(0.45);
    }
    else if (cargoIntake)
    {
      leftCargoIntake.set(0.45);
      rightCargoIntake.set(-0.45);
    }
    else
    {
      leftCargoIntake.set(-0.15);
      rightCargoIntake.set(0.15);
    }
   
    // -----------------------------------------------------------------
    if (-liftaxis <= 0.15 & -liftaxis >= -0.15 && bottomliftlimit == true)
    {
      liftMotor.set(ControlMode.PercentOutput, 0.0);
    }
    else if (-liftaxis <= 0.15 & -liftaxis >= -0.15 && bottomliftlimit == false)
    {
      liftMotor.set(ControlMode.PercentOutput, -0.16);
    }
    else
    {
      liftMotor.set(ControlMode.PercentOutput, -liftaxis);
    }

    // -----------------------------------------------------------------
    if (joy2.getRawButton(8) && joy2.getRawButton(9))
    {
      WclimbL.set(ControlMode.PercentOutput, 0.7);
      WclimbR.set(ControlMode.PercentOutput, 0.7);
    }
    else
    {
      WclimbL.set(ControlMode.PercentOutput, 0);
      WclimbR.set(ControlMode.PercentOutput, 0);
    }
    // -----------------------------------------------------------------
    if (joy2.getRawButton(10) == true)
    {
      climbL.set(ControlMode.PercentOutput, 1.0);
      climbR.set(ControlMode.PercentOutput, 1.0);
    }
    else if (joy2.getRawButton(11) == true)
    {
    climbL.set(ControlMode.PercentOutput, -1.0);
    climbR.set(ControlMode.PercentOutput, -1.0);
    }
    else
    {
      climbL.set(0);
      climbR.set(0);
    }

    // -----------------------------------------------------------------
  }

  @Override
  public void autonomousInit()
  {
    m_autoSelected = m_chooser.getSelected();
   
    System.out.println("Auto selected: " + m_autoSelected);
  }
  @Override
  public void autonomousPeriodic()
  {
    switch (m_autoSelected) 
    {
      case kCustomAuto:
        break;
      case kDefaultAuto:
      default:
          robotPeriodic();
        break;
    }
  }
  @Override
  public void teleopPeriodic()
  {
  }

  @Override
  public void testPeriodic() 
  {
  }

  public void USCorrection()
  {
    lineUS = ((USSLout - USSRout) / 500);
    if (Math.abs(USSLout - USSRout) <= 2)
    {
      driveRotation = 0;
      driveYAxis = -0.20;
    }
    else if (USSLout > USSRout)
    {
      driveRotation = 0.175 + lineUS;
    }
    else if (USSLout < USSRout)
    {
      driveRotation = -0.175 + lineUS;
    }
  }

  public void Autolift()
  {
    if (bottomliftlimit)
    {
      clear = true;
    }
    else
    {
      clear = false;
    }

    if (level1)
    {
      liftMotor.set(ControlMode.Position, 2000);
    }
    else if (level2)
    {
      liftMotor.set(ControlMode.Position, 4000);
    }
    else if (level3)
    {
      liftMotor.set(ControlMode.Position, 6000);
    }
    else
    {
      liftMotor.set(ControlMode.Position, 0);
    }
  }
}
