/**
 * This program is a sample program for the pneumatics test bed.
 * 
 * @author    Eli Wood, Zachary Sousa
 * @version   02 
 */


package frc.robot;

import edu.wpi.first.wpilibj.Compressor;

// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
// import com.ctre.phoenix.motorcontrol.can.VictorSPX;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX; // DONE: check if change is needed (comment this out?)

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import edu.wpi.first.wpilibj.XboxController; // added PG 29 May 2022
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Robot extends TimedRobot {

  XboxController operatorController = new XboxController(Constants.USB_OPERATOR_CONTROLLER);

  PowerDistribution pdp = new PowerDistribution();

  Compressor c = new Compressor(PneumaticsModuleType.CTREPCM);

  

  DoubleSolenoid ds1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.DS1_OUT, Constants.DS1_IN);
  DoubleSolenoid ds2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.DS2_OUT, Constants.DS2_IN);

  @Override
  public void robotInit() {
    ds1.set(Value.kOff);
    ds2.set(Value.kOff);
    reportStatus();

    c.enableDigital();
    System.out.println("Compressor status: " + c.enabled());

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
    if (operatorController.getRawButtonPressed(4) == true) { //y
      ds1.toggle();
    }
    if (operatorController.getRawButtonPressed(1) == true) { //x
      ds2.toggle();
    }
    if (operatorController.getRawButtonPressed(3) == true) { //b
      ds1.set(Value.kOff);
      ds2.set(Value.kOff);
    }
    if (operatorController.getRawButtonPressed(2) == true) { //a
      ds1.set(Value.kReverse);
      ds2.set(Value.kReverse);
    }
    
    /*
    if (operatorController.getRawButtonPressed(3) == true) { //b
      ds1.set(Value.kOff);
    }
    if (operatorController.getRawButtonPressed(4) == true) { //y
      ds1.set(Value.kReverse);
    }
    if (operatorController.getRawButtonPressed(8) == true) { //rt
      ds1.set(Value.kForward);
    }


    if (operatorController.getRawButtonPressed(2) == true) { //a
      ds2.set(Value.kOff);
    }
    if (operatorController.getRawButtonPressed(1) == true) { //x
      ds2.set(Value.kReverse);
    }
    if (operatorController.getRawButtonPressed(7) == true) { //lt
      ds2.set(Value.kForward);
    }
    */ 

  reportStatus();

  } // end teleopPeriodic()

  @Override
  public void disabledInit() {
    ds1.set(Value.kOff);
    ds2.set(Value.kOff);

    reportStatus();

  } // end disabledInit();
    

private void reportStatus() {

  /*SmartDashboard.putNumber("PDP Volts A V", pdp.getVoltage());
  SmartDashboard.putNumber("PDP Power A W", pdp.getTotalPower());
  SmartDashboard.putBoolean("Compresser status", c.enabled());*/



} // end reportStatus();

// *** end of changes PG 31 May 2022 ***


} // end class Robot