
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class Arm extends SubsystemBase {
  private static DoubleSolenoid d = new DoubleSolenoid(0,1);
  protected static Compressor c = new Compressor();
  public Arm() {
  }
  public void Pneumatic_Status(){
    c.setClosedLoopControl(true);
  }

  public void out(){
    d.set(DoubleSolenoid.Value.kReverse);  
  }

  public void in(){
    d.set(DoubleSolenoid.Value.kForward);
  }

  @Override
  public void periodic() {
  }
}
