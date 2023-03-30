/**
 * This program is a sample program for the shooter test bed.
 * 
 * @author    Eli Wood
 * @version   01 
 */


package frc.robot;

//import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
//import com.ctre.phoenix.motorcontrol.can.VictorSPX;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX; // DONE: check if change is needed (comment this out?)

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.XboxController; // added PG 29 May 2022
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PowerDistribution;

public class Robot extends TimedRobot {

  XboxController operatorController = new XboxController(Constants.USB_OPERATOR_CONTROLLER);

  TalonSRX shoot = new TalonSRX(Constants.MOTOR1);

  @Override
  public void robotInit() {


    reportStatus();

    
  } // end robotInit()

  @Override
  public void autonomousInit() {
    
  }
  @Override
  public void autonomousPeriodic() {
    
  } // end AutonomousInit()

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    reportStatus();

  } // end teleopInit()

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if (operatorController.getRawButton(1) == true) { //x
      shoot.set(ControlMode.PercentOutput, 1);
    } else if (operatorController.getRawButton(4) == true) { //y
      shoot.set(ControlMode.PercentOutput, -1);
    } else {
      shoot.set(ControlMode.PercentOutput, 0);
    }


  reportStatus();

  } // end teleopPeriodic()

  @Override
  public void disabledInit() {
    shoot.set(ControlMode.PercentOutput, 0.0);

    reportStatus();

  } // end disabledInit();
    

private void reportStatus() {

  /*SmartDashboard.putNumber("PDP Volts A V", pdp.getVoltage());
  SmartDashboard.putNumber("PDP Power A W", pdp.getTotalPower());
  SmartDashboard.putBoolean("Compresser status", c.enabled());*/



} // end reportStatus();

// *** end of changes PG 31 May 2022 ***


} // end class Robot