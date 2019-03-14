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

  MecanumDrive mecdrive = new MecanumDrive(talonFL, talonBL, talonFR, talonBR);

  Compressor compressor = new Compressor(0);
  Solenoid ejectorSolenoid = new Solenoid(6);

  @Override
  public void robotInit() 
  {
    // -----------------------------------------------------------------
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    // -----------------------------------------------------------------
    compressor.setClosedLoopControl(true);

    // -----------------------------------------------------------------
    AxisCamera camera2 = CameraServer.getInstance().addAxisCamera("10.20.39.11");
    UsbCamera camera1 = CameraServer.getInstance().startAutomaticCapture(0);
    camera1.setResolution(360, 240);
    camera1.setFPS(15);

    // -----------------------------------------------------------------
  }
  @Override
  public void robotPeriodic() 
  {
    // -----------------------------------------------------------------
    operator_ZAxis = joy2.getRawAxis(2);
    liftaxis = joy2.getRawAxis(1);

    // -----------------------------------------------------------------
    driveXAxis = joy1.getRawAxis(0);
    driveYAxis = joy1.getRawAxis(1);
    driveRotation = joy1.getRawAxis(4); 
    mecdrive.driveCartesian(driveXAxis , -driveYAxis , driveRotation);

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
      leftCargoIntake.set(-0.6);
      rightCargoIntake.set(0.6);
    }
    else if (cargoIntake)
    {
      leftCargoIntake.set(0.6);
      rightCargoIntake.set(-0.6);
    }
    else
    {
      leftCargoIntake.set(-0.15);
      rightCargoIntake.set(0.15);
    }
   
    // -----------------------------------------------------------------
    if (-liftaxis <= 0.15 & -liftaxis >= -0.15)
    {
      liftMotor.set(ControlMode.PercentOutput, 0.0);
    }
    else
    {
      liftMotor.set(ControlMode.PercentOutput, -liftaxis);
    }

    // -----------------------------------------------------------------
    if (joy2.getRawButton(8) & joy2.getRawButton(9))
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
}
