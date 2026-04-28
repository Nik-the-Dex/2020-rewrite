package frc.robot.SensorInputs;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class NavX
{
    public final AHRS ahrs;

    public double yaw;
    public double angle;
    public double pitch;
    public double roll;

    public NavX()
    {
        ahrs = new AHRS(NavXComType.kMXP_SPI);
    }

    public void updateAHRS()
    {
        yaw   = ahrs.getYaw();
        angle = ahrs.getAngle();
        pitch = ahrs.getPitch();
        roll  = ahrs.getRoll();

        SmartDashboard.putNumber("angle", angle);
    }

    public void resetAngle()
    {
        ahrs.zeroYaw();
    }
}
