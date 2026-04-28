package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbMech extends SubsystemBase
{
    private final SparkMax    climbMotor1    = new SparkMax(frc.robot.Constants.CLIMBMECH_MOTOR1, MotorType.kBrushless);
    public  final SparkMax    weirdClimbMotor = new SparkMax(frc.robot.Constants.WEIRD_CLIMBMECH_MOTOR, MotorType.kBrushed);
    private final RelativeEncoder  climbEncoder   = climbMotor1.getEncoder();

    public final DoubleSolenoid hangerSolenoid = new DoubleSolenoid(
        frc.robot.Constants.PCM2,
        PneumaticsModuleType.CTREPCM,
        frc.robot.Constants.HANGER_FWD,
        frc.robot.Constants.HANGER_REV
    );

   
    public ClimbMech()
    {
        
      

        climbEncoder.setPosition(0);
    }

    public void extendHanger()
    {
        if (hangerSolenoid.get() == Value.kReverse)
        {
            hangerSolenoid.set(Value.kForward);
        }
        else
        {
            hangerSolenoid.set(Value.kReverse);
        }
    }

    public void setClimbMotors(double speed)
    {
        climbMotor1.set(speed);
    }

    public void climb()
    {
        if (climbEncoder.getPosition() < 106)
        {
            setClimbMotors(0.5);
        }
        else
        {
            setClimbMotors(0);
        }
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("climbEncoder",   climbEncoder.getPosition());
        SmartDashboard.putBoolean("hangerExtended", hangerSolenoid.get() == Value.kForward);
    }
}
