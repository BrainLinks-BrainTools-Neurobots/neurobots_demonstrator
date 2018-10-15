package application;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.RoboticsAPIContext;
import com.kuka.roboticsAPI.userInterface.ServoMotionUtilities;
import java.io.File;

public class RobotApplication {
	private Controller lbrController_LBR4;
	private LBR lbrLBRRight;
	private String path;

	public void setPath(String p) {
		path = p;
	}

	public void initialize() {
	    RoboticsAPIContext context;
	    File file = new File(path + "/RoboticsAPI.config.xml");
	    context = RoboticsAPIContext.createFromFile(file);   
		lbrController_LBR4 = context.getController("Sunrise");
		lbrLBRRight = ServoMotionUtilities.locateLBR(context);
	}

	public void run() {
	    //lbrLBRRight.move(ptp(0., Math.toRadians(30.), 0., -Math.toRadians(60.), 0.,
        //        -Math.toRadians(60.), 0.).setJointVelocityRel(0.1));
	    lbrLBRRight.move(ptpHome().setJointVelocityRel(0.1));
	}

	/**
	 * Auto-generated method stub. Do not modify the contents of this method.
	 */
	public static void main(String[] args) {
		if (args.length < 1) {
			System.out.println("Parameters needed: [path]");
			return;
		}

		RobotApplication app = new RobotApplication();
		app.setPath(args[0]);
		app.initialize();
		app.run();
	}
}
