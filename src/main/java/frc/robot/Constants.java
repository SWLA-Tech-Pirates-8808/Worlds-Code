
package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;




/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  
  public static class OperatorConstants {
    public static final double kDriverControllerPort = 1;
  }

  public static class motorConstants {
    public static final CANSparkMax leftMotorFront = new CANSparkMax(1, MotorType.kBrushed);
    public static final CANSparkMax leftMotorBack = new CANSparkMax(2, MotorType.kBrushed);
    public static final CANSparkMax rightMotorFront = new CANSparkMax(3, MotorType.kBrushed);
    public static final CANSparkMax rightMotorBack = new CANSparkMax(4, MotorType.kBrushed);
  }

  public static class controllerConstants {
    public static final XboxController controllerVROOMVROOM = new XboxController(0);
    public static final XboxController controllerTHECLAWWWWW = new XboxController(1);
  }

  public static class speedConstants {
    public static final double kMaxSpeed = 1.5;
    public static final double armSpep = 1; 
    public static final double aSpep = 1;
    public static final double aMovingSpep = 0.5;
    public static final double logaspep = 2.0;
    public static final double mecanumSpep = 0.75;
    public static final double turnspep = 0.75;
    public static final double stendoSpep = 0.75;
    public static final double stendoSpep2 = 0.25;

  }
}

