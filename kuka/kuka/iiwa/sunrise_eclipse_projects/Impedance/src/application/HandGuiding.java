package application;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;
import static com.kuka.roboticsAPI.motionModel.MMCMotions.*;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.motionModel.HandGuidingMotion;
import com.kuka.roboticsAPI.motionModel.controlModeModel.HandGuidingControlMode;

public class HandGuiding extends RoboticsAPIApplication {

	private Controller kuka_Sunrise_Cabinet_1;
	private LBR lbr_iiwa_14_R820_1;
	private Tool gripper;

	public void initialize() {
		kuka_Sunrise_Cabinet_1 = getController("KUKA_Sunrise_Cabinet_1");
		lbr_iiwa_14_R820_1 = (LBR) getDevice(kuka_Sunrise_Cabinet_1,
				"LBR_iiwa_14_R820_1");
		gripper = getApplicationData().createFromTemplate("SchunkWSG50");
		gripper.attachTo(lbr_iiwa_14_R820_1.getFlange());
	}

	public void run() {
		getLogger().info("to start");
		lbr_iiwa_14_R820_1.setESMState("1");
//		lbr_iiwa_14_R820_1.move(ptp(0, 0, 0, -Math.PI/2, 0, Math.PI/2, 0).setJointVelocityRel(0.2));
		lbr_iiwa_14_R820_1.move(ptp(-0.81, 0.96, -0.35, -1.46, 1.58, -1.12, 0.04).setJointVelocityRel(0.2));
		getLogger().info("hand guiding active");
		lbr_iiwa_14_R820_1.setESMState("2");
		HandGuidingMotion motion = handGuiding();
		HandGuidingControlMode mode = new HandGuidingControlMode();
		mode.setMaxJointSpeed(100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0);
		motion.setMode(mode);
		lbr_iiwa_14_R820_1.move(motion);
		getLogger().info(motion.getParams().toString());
		getLogger().info("hand guiding finished");
		lbr_iiwa_14_R820_1.setESMState("1");
	}

	/**
	 * Auto-generated method stub. Do not modify the contents of this method.
	 */
	public static void main(String[] args) {
		HandGuiding app = new HandGuiding();
		app.getLogger().warn("called");
		app.runApplication();
	}

}
