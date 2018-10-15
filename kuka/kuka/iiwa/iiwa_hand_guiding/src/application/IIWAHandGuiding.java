package application;

import java.io.File;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;
import static com.kuka.roboticsAPI.motionModel.MMCMotions.*;

import com.kuka.roboticsAPI.RoboticsAPIContext;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.userInterface.ServoMotionUtilities;

public class IIWAHandGuiding {
	public static void main(String args[]) throws Exception {
		if (args.length != 1) {
			System.out.println("Parameters needed: [path]");
			return;
		}

		RoboticsAPIContext context;
		File configFile = new File(args[0] + "/RoboticsAPI.config.xml");
		context = RoboticsAPIContext.createFromFile(configFile);
		LBR robot = ServoMotionUtilities.locateLBR(context);
		robot.move(handGuiding());
	}
}
