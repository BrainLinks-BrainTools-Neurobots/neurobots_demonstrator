package application;

import java.io.File;
import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.List;

import com.kuka.roboticsAPI.RoboticsAPIContext;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.Vector;
import com.kuka.roboticsAPI.sensorModel.ForceSensorData;
import com.kuka.roboticsAPI.userInterface.ServoMotionUtilities;

import java.lang.Thread;

public class UDPIIWAJointStatePublisher extends Thread {
	IIWAJointStatePublisherDaemon jointStatePublisher;
	public class IIWAJointStatePublisherDaemon {
		private int port;
		private int portOut;
		private LBR lbr;
		private int counter;
		private String path;
		private Tool gripper;
		
		IIWAJointStatePublisherDaemon(String path) {
			this.path = path;
		}

		public void initialize() {
			RoboticsAPIContext context;
			File configFile = new File(path + "/RoboticsAPI.config.xml");
			context = RoboticsAPIContext.createFromFile(configFile);
			lbr = ServoMotionUtilities.locateLBR(context);
			gripper = ServoMotionUtilities.createToolFromTemplate("SchunkWSG50", path + "/RoboticsAPI.data.xml", context);
			gripper.attachTo(lbr.getFlange());
			
			counter = 0;
		}

		public void initNetwork(int port, int portOut) {
			this.port = port;
			this.portOut = portOut;
		}

		public void run() {
			try {
				DatagramSocket serverSocket = new DatagramSocket(port);
				List<ObjectFrame> frames;
				frames = lbr.getAllFrames();
				
				while (true) {
					// Get joint states
					JointPosition joints = lbr.getCurrentJointPosition();
					double[] torque = lbr.getMeasuredTorque().getTorqueValues();
					ForceSensorData forceData = lbr.getExternalForceTorque(gripper.getDefaultMotionFrame());
					
					// Write joint position into buffer
					byte[] sendData = new byte[1024];
					ByteBuffer bf = ByteBuffer.wrap(sendData);
					bf.order(ByteOrder.LITTLE_ENDIAN);
					
					// Init header
					bf.putInt(counter);
					bf.putShort((short)3);

					// Insert joint position
					for(int i = 0; i < 7; i++) {
						bf.putDouble(joints.get(i));
					}
					
					// Insert torque
					for(int i = 0; i < 7; i++) {
						bf.putDouble(torque[i]);
					}
					
					//Insert flange force and torque
					Vector flangeForce = forceData.getForce();
					Vector flangeTorque = forceData.getTorque();
					bf.putDouble(flangeForce.getX());
					bf.putDouble(flangeForce.getY());
					bf.putDouble(flangeForce.getZ());
					
					bf.putDouble(flangeTorque.getX());
					bf.putDouble(flangeTorque.getY());
					bf.putDouble(flangeTorque.getZ());
					
					// Insert time
					bf.putLong(System.currentTimeMillis());
					
					// Create package and send it
					DatagramPacket sendPacket = new DatagramPacket(sendData, sendData.length, InetAddress.getByName("localhost"), portOut);
					serverSocket.send(sendPacket);
					sleep(10);
				}
			} catch (SocketException e) {
				e.printStackTrace();
			} catch (IOException e) {
				e.printStackTrace();
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			
		}
	}
	
	public void init(int port, int portOut, String path) {
		jointStatePublisher = new IIWAJointStatePublisherDaemon(path);
		jointStatePublisher.initialize();
		jointStatePublisher.initNetwork(port, portOut);
	}
	
	@Override
	public void run() {
		// Let the application run
		jointStatePublisher.run();
	}

	public static void main(String args[]) throws Exception {
		if (args.length != 2) {
			System.out.println("Parameters needed: [port] [portOut]");
			return;
		}
		UDPIIWAJointStatePublisher controller = new UDPIIWAJointStatePublisher();
		controller.init(Integer.parseInt(args[0]), Integer.parseInt(args[1]), args[2]);
	}
}
