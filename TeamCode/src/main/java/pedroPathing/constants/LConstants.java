package pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class LConstants {
    static {
        TwoWheelConstants.forwardTicksToInches = 0.0029692837461563278;
        TwoWheelConstants.strafeTicksToInches = 0.0029513343909821914;
        TwoWheelConstants.forwardY = -5.875;
        TwoWheelConstants.strafeX = -4.875;
        TwoWheelConstants.forwardEncoder_HardwareMapName = "bl";
        TwoWheelConstants.strafeEncoder_HardwareMapName = "fr";
        TwoWheelConstants.forwardEncoderDirection = Encoder.REVERSE;
        TwoWheelConstants.strafeEncoderDirection = Encoder.FORWARD;
        TwoWheelConstants.IMU_HardwareMapName = "imu";
        TwoWheelConstants.IMU_Orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);
    }
}




