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

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.RoboticsAPIContext;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.sensorModel.ForceSensorData;
import com.kuka.roboticsAPI.userInterface.ServoMotionUtilities;







//Thread stuff
import java.lang.Thread;

public class UDPLBRJointStatePublisher extends Thread {
	LBRJointStatePublisherDaemon jointStatePublisher;
	public class LBRJointStatePublisherDaemon {
		private int port;
		private int portOut;
		private LBR theLBR;
		private int counter;
		private String path;
		
		LBRJointStatePublisherDaemon(String path) {
			this.path = path;
		}

		public void initialize() {
			RoboticsAPIContext context;
			File configFile = new File(path + "/RoboticsAPI.config.xml");
			context = RoboticsAPIContext.createFromFile(configFile);
			theLBR = ServoMotionUtilities.locateLBR(context);
			
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
				frames = theLBR.getAllFrames();
				
				while (true) {
					// Get joint states
					JointPosition joints = theLBR.getCurrentJointPosition();
					double[] torque = theLBR.getMeasuredTorque().getTorqueValues();
					
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
					
					// Insert force for every frame
					//for(ObjectFrame frame : frames) {
					//	ForceSensorData forceData = theLBR.measureForceTorque(frame);
					//	bf.putDouble(forceData.getForce().getX());
					//	bf.putDouble(forceData.getForce().getY());
					//	bf.putDouble(forceData.getForce().getZ());
					//}
					
					// Insert time
					bf.putLong(System.currentTimeMillis());
					
					// Create package and send it
					DatagramPacket sendPacket = new DatagramPacket(sendData, sendData.length, InetAddress.getByName("localhost"), portOut);
					serverSocket.send(sendPacket);
					sleep(10);
				}
				
				//serverSocket.close();
				//serverSocketOut.close();
			} catch (SocketException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			
		}
	}
	
	public void init(int port, int portOut, String path) {
		jointStatePublisher = new LBRJointStatePublisherDaemon(path);
		jointStatePublisher.initialize();
		jointStatePublisher.initNetwork(port, portOut);
	}
	
	@Override
	public void run() {
		// Let the application run
		//jointStatePublisher.runApplication();
		jointStatePublisher.run();
	}

	public static void main(String args[]) throws Exception {
		if (args.length != 3) {
			System.out.println("Parameters needed: [port] [portOut]");
			return;
		}
		UDPLBRJointStatePublisher controller = new UDPLBRJointStatePublisher();
		controller.init(Integer.parseInt(args[0]), Integer.parseInt(args[1]), args[2]);
	}
}
