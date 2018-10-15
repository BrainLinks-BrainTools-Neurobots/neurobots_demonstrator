package application;

import java.io.File;
import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.lang.Thread;
import java.lang.IllegalStateException;

import com.kuka.common.ThreadUtil;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import com.kuka.roboticsAPI.RoboticsAPIContext;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.executionModel.CommandInvalidException;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.motionModel.ISmartServoRuntime;
import com.kuka.roboticsAPI.motionModel.SmartServo;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.userInterface.ServoMotionUtilities;

public class UDPIIWAController {
	IIWADaemon armServo;
	UDPDaemon udpClient;

	private final Lock _mutex = new ReentrantLock(true);
	private int mode = -1;
	private JointPosition aPosition;
	private double x, y, z, rotX, rotY, rotZ;
	volatile private double jointVelocity;
	volatile private double jointAcc;
	private List<double[]> joints = new ArrayList<double[]>();
	private List<double[]> velos = new ArrayList<double[]>();
	private boolean stop = false;

	// -------------------------------------------------------------------------------------------------------

	public class IIWADaemon extends Thread {
		private LBR theLBR;

		IIWADaemon(String path) {
			RoboticsAPIContext context;
			File configFile = new File(path + "/RoboticsAPI.config.xml");
			context = RoboticsAPIContext.createFromFile(configFile);
			theLBR = ServoMotionUtilities.locateLBR(context);

/**
for(int i = 0; i < 5; ++i) {
			IMotionContainer motion = theLBR.moveAsync(ptp(new JointPosition(0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0)).setJointVelocityRel(0.5)
										.setJointAccelerationRel(0.2));
	while(motion != null && motion.getState().isFinished()) {
		try {
    Thread.sleep(10);                 //1000 milliseconds is one second.
} catch(InterruptedException ex) {
    Thread.currentThread().interrupt();
}
}

motion = theLBR.moveAsync(ptp(new JointPosition(0, 0, 0, -0.5, 0, 0, -0.0)).setJointVelocityRel(0.1)
										.setJointAccelerationRel(0.1));
	while(motion != null && motion.getState().isFinished()) {
		try {
    Thread.sleep(10);                 //1000 milliseconds is one second.
} catch(InterruptedException ex) {
    Thread.currentThread().interrupt();
}
}
} */
			
		}

		public void initSpeedCommands(double jointVel, double jointAc) {
			_mutex.lock();
			jointVelocity = jointVel;
			jointAcc = jointAc;
			_mutex.unlock();
		}

		@Override
		public void run() {
			SmartServo aSmartServoMotion = new SmartServo(theLBR.getCurrentJointPosition());
			aSmartServoMotion.setJointAccelerationRel(1.0);
			aSmartServoMotion.setSpeedTimeoutAfterGoalReach(0);
			aSmartServoMotion.setTimeoutAfterGoalReach(0.1);
			IMotionContainer motion = null;
			JointPosition currentGoal = null;
			while (true) {
				try {
					int modus;
					_mutex.lock();
					modus = mode;
					_mutex.unlock();
					if (modus > -1) {
						if (modus == 0) {
							_mutex.lock();
							if (motion == null) {
								currentGoal = aPosition;
								motion = theLBR.moveAsync(ptp(aPosition).setJointVelocityRel(jointVelocity)
										.setJointAccelerationRel(jointAcc));
							}

							_mutex.unlock();
							// wait until finished
							while (true) {
								_mutex.lock();
								// new goal

								if (currentGoal != aPosition) {
									if (motion != null) {
										motion.cancel();	
										// Send finished message
										// Send message for synchronous move
										byte[] sendData = new byte[2];
										ByteBuffer bfOut = ByteBuffer.wrap(sendData);
										bfOut.order(ByteOrder.LITTLE_ENDIAN);
										bfOut.putShort(0, (short) 1);
										sendFinishedMessage(sendData);
									}

									currentGoal = aPosition;
									motion = theLBR.moveAsync(ptp(currentGoal).setJointVelocityRel(jointVelocity)
											.setJointAccelerationRel(jointAcc));
								}
								/*if (stop) {
									if (motion != null) {
										motion.cancel();									
									}
									_mutex.unlock();
									break;
								}*/

								if (motion != null && motion.getState().isFinished())
								{
									motion = null;
									System.out.println("motion finished");
									_mutex.unlock();
									break;
								}


								_mutex.unlock();

								try {
								Thread.sleep(10); // 1ms
								}
								catch(InterruptedException ex) {
								}
							}

							// move to the current position if stop is true (and
							// so stop the robot)
							/*_mutex.lock();
							if (stop) {
								theLBR.moveAsync(ptp(theLBR.getCurrentJointPosition())
										.setJointVelocityRel(jointVelocity).setJointAccelerationRel(jointAcc));
							}
							_mutex.unlock();*/
						} else if (modus == 1) {
							Frame frame = new Frame(theLBR.getCurrentCartesianPosition(theLBR.getFlange()));
							_mutex.lock();
							frame.setX(x).setY(y).setZ(z).setAlphaRad(rotX).setBetaRad(rotY).setGammaRad(rotZ);
							theLBR.move(
									ptp(frame).setJointVelocityRel(jointVelocity).setJointAccelerationRel(jointAcc));
							_mutex.unlock();
						} else if (modus == 4) {
							// Execute path
							_mutex.lock();
							aSmartServoMotion.setJointVelocityRel(jointVelocity);
							_mutex.unlock();
							// aSmartServoMotion.setBlendingRel(0.5);
							// aSmartServoMotion.setMinimumSynchronizationTime(0.001);

							theLBR.moveAsync(aSmartServoMotion);
							ISmartServoRuntime theServoRuntime = aSmartServoMotion.getRuntime();
							// theServoRuntime.setSpeedModeExactStop();
							theServoRuntime.activateVelocityPlanning(true);
							System.out.println("Start path trajectory");

							int jointsSize = 0;
							_mutex.lock();
							jointsSize = joints.size();
							_mutex.unlock();
							double[] joint = new double[7];
							double[] velo = new double[7];
							while (jointsSize > 0) {
								_mutex.lock();
								if (joints.isEmpty()) {
									jointsSize = joints.size();
									_mutex.unlock();
									break;
								}
								joint = joints.remove(0);
								velo = velos.remove(0);
								jointsSize = joints.size();
								_mutex.unlock();

								JointPosition aPosition = new JointPosition(joint[0], joint[1], joint[2], joint[3],
										joint[4], joint[5], joint[6]);
								// JointPosition aSpeed = new JointPosition(velo[0], velo[1], velo[2], velo[3], velo[4],
								// 		velo[5], velo[6]);
                                                                JointPosition aSpeed = new JointPosition(0, 0, 0 ,0 ,0 ,0, 0);
								theServoRuntime.setDestination(aPosition, aSpeed);
								if (jointsSize == 0) {
									while (!theServoRuntime.isDestinationReached()) {
										ThreadUtil.milliSleep(1);
										_mutex.lock();
										jointsSize = joints.size();
										_mutex.unlock();

										if (jointsSize > 0) {
											double jointVal = 0;
											for (int k = 0; k < 7; k++) {
												jointVal += Math.abs(theLBR.getCurrentJointPosition().get(k)
														- theServoRuntime.getCurrentJointDestination().get(k));
											}

											if (jointVal < 0.1) {
												break;
											}
										}
									}
								} else {
									theServoRuntime.updateWithRealtimeSystem();
									double duration = theServoRuntime.getRemainingTime();
									double stop = Math.max(duration * 0.2, 0.6);
									// System.out.println(duration);

									/*
									 * while(!theServoRuntime.
									 * isDestinationReached()) { double jointVal
									 * = 0; for(int k = 0; k < 7; k++) {
									 * jointVal += Math.abs(theServoRuntime.
									 * getCommandResultOfInterpolation().get(k)
									 * - theServoRuntime.
									 * getCurrentJointDestination().get(k)); }
									 * 
									 * if(jointVal < 0.3) { break; }
									 * theServoRuntime.updateWithRealtimeSystem(
									 * ); }
									 */

									// double start = System.nanoTime();
									// int counter = 0;
									while (theServoRuntime.getRemainingTime() > stop) {
										// counter++;

										// ThreadUtil.milliSleep(1);
										// System.out.println(theServoRuntime.getRemainingTime());
										theServoRuntime.updateWithRealtimeSystem();
									}
									// double end = System.nanoTime();
									/*
									 * System.out.println(start);
									 * System.out.println(end);
									 * System.out.println((end-start));
									 * System.out.println(counter);
									 * System.out.println(1.0/(((end-start)/1.
									 * 0e9) / counter));
									 */
								}

							}
							theServoRuntime.stopMotion();
							ServoMotionUtilities.acknowledgeError(theLBR);
							System.out.println("Trajectory finished");
						}

						// Send finished message
						// Send message for synchronous move
						byte[] sendData = new byte[2];
						ByteBuffer bfOut = ByteBuffer.wrap(sendData);
						bfOut.order(ByteOrder.LITTLE_ENDIAN);
						bfOut.putShort(0, (short) 1);
						sendFinishedMessage(sendData);

						_mutex.lock();
						mode = -1;
						_mutex.unlock();
					} else {
						ThreadUtil.milliSleep(1);
					}
				} catch (IllegalStateException e) {
					System.out.println("IllegalStateException: " + e.getMessage());
					e.printStackTrace();
				} catch (CommandInvalidException e) {
					System.out.println("CommandInvalidException: " + e.getMessage());
					e.printStackTrace();
				}
			}
		}
	}

	// -------------------------------------------------------------------------------------------------------

	public class UDPDaemon extends Thread {
		private int port;
		private DatagramSocket serverSocket;

		UDPDaemon() {
		}

		public void initNetwork(int port) {
			this.port = port;
		}

		public void run() {
			try {
				serverSocket = new DatagramSocket(port);
				while (true) {
					try {
						byte[] receiveData = new byte[65536];
						System.out.println("Waiting for new IIWA command");
						DatagramPacket receivePacket = new DatagramPacket(receiveData, receiveData.length);
						serverSocket.receive(receivePacket);
						System.out.println("Command received. Now starting...");
						ByteBuffer bf = ByteBuffer.wrap(receivePacket.getData());
						bf.order(ByteOrder.LITTLE_ENDIAN);

						int counter = bf.getInt(0);
						int modus = bf.getShort(4);

						if (modus == 0) {
							double j1 = bf.getDouble(6);
							double j2 = bf.getDouble(14);
							double j3 = bf.getDouble(22);
							double j4 = bf.getDouble(30);
							double j5 = bf.getDouble(38);
							double j6 = bf.getDouble(46);
							double j7 = bf.getDouble(54);
							long tstamp = bf.getLong(62);

							// Create message
							String header = "New Message with joints:\nTimestamp: " + tstamp + "\nCounter: " + counter
									+ "\n";
							String joints = "Joint 1: " + j1 + "\nJoint 2: " + j2 + "\nJoint 3: " + j3 + "\nJoint 4: "
									+ j4 + "\nJoint 5: " + j5 + "\nJoint 6: " + j6 + "\nJoint 7: " + j7 + "\n";

							System.out.println(header + joints);
							_mutex.lock();
							aPosition = new JointPosition(j1, j2, j3, j4, j5, j6, j7);
							stop = false;
							_mutex.unlock();
						} else if (modus == 1) {
							_mutex.lock();
							x = bf.getDouble(6);
							y = bf.getDouble(14);
							z = bf.getDouble(22);
							rotX = bf.getDouble(30);
							rotY = bf.getDouble(38);
							rotZ = bf.getDouble(46);
							long tstamp = bf.getLong(54);

							// Create message
							String header = "New Message with cartesian:\nTimestamp: " + tstamp + "\nCounter: "
									+ counter + "\n";
							String cartesian = "X: " + x + "\nY: " + y + "\nZ: " + z + "\nAlpha: " + rotX + "\nBeta: "
									+ rotY + "\nGamma: " + rotZ + "\n";
							_mutex.unlock();

							System.out.println(header + cartesian);
						} else if (modus == 3) {
							double velTheta = bf.getDouble(30);
							double accTheta = bf.getDouble(62);
							_mutex.lock();
							jointVelocity = velTheta;
							jointAcc = accTheta;
							_mutex.unlock();

							// Create message
							String header = "New Message with speed:\nTimestamp: " + 0 + "\nCounter: " + counter + "\n";
							String cartesian = "X: " + velTheta + "\nY: " + accTheta + "\n";

							System.out.println(header + cartesian);
						} else if (modus == 4) {

							// Read long movit message
							int pathLength = bf.getInt(6);
							System.out.println("Got msg with path of size " + pathLength);
							int index = 10; // For int size
							int indI = 0;
							double[][] timeToStart = new double[pathLength][7];
							int currentPathLength = pathLength;
							if (pathLength > 350) {
								currentPathLength = 350;
							}
							_mutex.lock();
							joints.clear();
							velos.clear();
							while (true) {
								for (int k = 0; k < currentPathLength; k++) {
									// Read 7 joint positions
									double joint[] = new double[7];
									for (int j = 0; j < 7; j++) {
										joint[j] = bf.getDouble(index);
										index += 8;
									}
									joints.add(joint);
								}
								index = 10 + 350 * 8 * 7;
								for (int i = indI; i < currentPathLength; i++) {
									// Read 7 velos positions
									double velo[] = new double[7];
									for (int j = 0; j < 7; j++) {
										velo[j] = bf.getDouble(index);
										// Max Velocity is 1.963
										double max_velocity = 1.963;
										if (velo[j] > max_velocity) {
											velo[j] = max_velocity;
										} else if (velo[j] < -max_velocity) {
											velo[j] = -max_velocity;
										}
										index += 8;
									}
									velos.add(velo);
								}
								index = 10 + 700 * 8 * 7;
								for (int i = indI; i < currentPathLength; i++) {
									// Read 7 accels positions
									for (int j = 0; j < 7; j++) {
										timeToStart[i][j] = bf.getDouble(index);
										index += 8;
									}
								}

								// If more joints are available, read them
								if (pathLength > 350) {
									serverSocket.receive(receivePacket);
									bf = ByteBuffer.wrap(receivePacket.getData());
									pathLength = bf.getInt(6);
								} else {
									break;
								}

								indI += 350;
								index = 10;
							}
							_mutex.unlock();
						} else if (modus == 5) {
							_mutex.lock();
							if (mode == 0 || mode == 1) {
								stop = true;
							} else if (mode == 4) {
								joints.clear();
								velos.clear();
							} else {
								System.out.println("Nothing to stop. Give new command.");
							}
							modus = -1;
							_mutex.unlock();
						} else {
							System.out.println("No correct mode given. " + "Retry mode 0 for joint angles or 1 for "
									+ "cartesian coords or 3 to set acc and vel. Mode given: " + modus);
							modus = -1;
						}
						_mutex.lock();
						if (modus != 3)
							mode = modus;
						_mutex.unlock();
					} catch (SocketException e) {
						e.printStackTrace();
					} catch (CommandInvalidException e) {
						System.out.println("Invalid Command! IIWA stopped. Send a new command...");
					}
				}
			} catch (SocketException e) {
				System.out.println("Received timeout too early...");
				e.printStackTrace();
			} catch (IOException e) {
				e.printStackTrace();
			}
		}
	}

	// -------------------------------------------------------------------------------------------------------

	public void sendFinishedMessage(byte[] sendData) {
		DatagramPacket sendPacket;
		try {
			sendPacket = new DatagramPacket(sendData, sendData.length, InetAddress.getByName("localhost"),
					udpClient.port + 1);
			udpClient.serverSocket.send(sendPacket);
		} catch (UnknownHostException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		}

		System.out.println("send");
	}

	public void init(int port, double velocity, double acc, String path) {
		udpClient = new UDPDaemon();
		udpClient.initNetwork(port);
		armServo = new IIWADaemon(path);
		armServo.initSpeedCommands(velocity, acc);
		udpClient.start();
		armServo.start();
	}

	public static void main(String args[]) throws Exception {
		if (args.length != 3) {
			System.out.println("Parameters needed: [Port] [joint_velocity] [path]");
			return;
		}
		UDPIIWAController controller = new UDPIIWAController();
		controller.init(Integer.parseInt(args[0]), Double.parseDouble(args[1]), Double.parseDouble(args[1]), args[2]);
	}
}
