
package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.Quaternion;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.drive.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.*;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.hal.can.CANMessageNotAllowedException;
import edu.wpi.first.wpilibj.AnalogOutput;
import edu.wpi.first.wpilibj.I2C;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


public class Robot extends TimedRobot 
{
  
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  double xaxis;
  double rotation;

  double position;
  double velocity;
  boolean clear;
  boolean button = false;
  double controlmode;
  private static final Joystick joy1 = new Joystick(0);

  private static final TalonSRX motor = new TalonSRX(11); 


  @Override
  public void robotInit() 
  {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    motor.set(ControlMode.PercentOutput, 0);
    motor.configFactoryDefault();
    motor.setNeutralMode(NeutralMode.Brake);
    motor.setInverted(true);

    
    motor.configNominalOutputForward(0, 30);
		motor.configNominalOutputReverse(0, 30);
		motor.configPeakOutputForward(1, 30);
		motor.configPeakOutputReverse(-1, 30);
  }

 
  @Override
  public void robotPeriodic() 
  {
  xaxis = joy1.getRawAxis(1);
  rotation = joy1.getRawAxis(4);
  clear = joy1.getRawButton(1);
  button = joy1.getRawButton(2);
  motor.setSensorPhase(true);
  motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
  motor.config_kP(0, 0.01, 30);
  motor.config_kI(0, 0.25, 30);
  motor.config_kD(0, 0.015, 30);
  motor.config_kF(0, 0.1, 30);
  motor.configClosedLoopPeriod(0, 1, 30);
  motor.configClearPositionOnQuadIdx(clear, 20);
  position = motor.getSelectedSensorPosition();
  velocity = motor.getSelectedSensorVelocity();
  SmartDashboard.putNumber("Postion", position);
  SmartDashboard.putNumber("Velocity", velocity);
  LiveWindow.addActuator("Component", "Motor", motor);
}
  @Override
  public void autonomousInit() 
  {
    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);
  }

  
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        
        break;
      case kDefaultAuto:
      default:
      
        break;
    }
  }

 
  @Override
  public void teleopPeriodic() 
  {

  }


  @Override
  public void testPeriodic() {
  }
}
