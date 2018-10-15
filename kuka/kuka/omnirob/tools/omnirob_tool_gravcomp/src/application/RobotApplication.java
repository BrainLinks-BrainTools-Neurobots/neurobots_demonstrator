package application;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.RoboticsAPIContext;
import com.kuka.roboticsAPI.userInterface.ServoMotionUtilities;
import java.io.File;
import com.kuka.roboticsAPI.executionModel.ICommandContainer;
import com.kuka.roboticsAPI.commandModel.Instruction;
import com.kuka.roboticsAPI.motionModel.GravComp;
import com.kuka.roboticsAPI.geometricModel.PhysicalObject;
import sun.misc.Signal; 
import sun.misc.SignalHandler;

import application.Quit;

public class RobotApplication {
	private Controller lbrController_LBR4;
	private LBR lbrLBRRight;
	private PhysicalObject gripper;
	private double massOfGripper;
	private String path;

    public void setMass(double mass) {
		massOfGripper = mass;
	}

 	public void setPath(String p) {
		path = p;
	}

	public void initialize() {
	    RoboticsAPIContext context;
	    File file = new File(path + "/RoboticsAPI.config.xml");
	    context = RoboticsAPIContext.createFromFile(file);   
		lbrController_LBR4 = context.getController("Sunrise");
		lbrLBRRight = ServoMotionUtilities.locateLBR(context);
		
		double translationOfTool[] = { 0, 0, 0 };
		//double mass = 1.5;
		double centerOfMassInMillimeter[] = { 0, 0, 100.0 };
		
		gripper = ServoMotionUtilities.createTool(lbrLBRRight, "WSG", translationOfTool, massOfGripper, centerOfMassInMillimeter);
		lbrLBRRight.getLoadData().fill(gripper.getLoadData());
	}

	public void run() {
	    ICommandContainer cont3 = lbrLBRRight.getController().getExecutionService().beginExecution(new Instruction(new GravComp(), lbrLBRRight), null);
	    System.out.println("The mass of the gripper is: " + lbrLBRRight.getLoadData().getMass());
	    System.out.println("Execute the program with additional parameter to specify the gripper mass.");
	    //Quit quit = new Quit("GravComp");
		while(true)
		{
			try {
			Thread.sleep(100);
			}catch(InterruptedException ex){
			}
		}
	}

	/**
	 * Auto-generated method stub. Do not modify the contents of this method.
	 */
	public static void main(String[] args) {
		Signal.handle(new Signal("INT"), new SignalHandler() {
		      // Signal handler method
		      public void handle(Signal signal) {
			System.exit(0);
		      }
		    });

		RobotApplication app = new RobotApplication();
		
		if (args.length < 1) {
			System.out.println("Parameters needed: [path] (mass)");
			return;
		}

		if(args.length == 1) {
			app.setMass(0);
		}
		else
		{
			app.setMass(Double.parseDouble(args[1]));
		}

		app.setPath(args[0]);

		app.initialize();
		app.run();
	}
}
