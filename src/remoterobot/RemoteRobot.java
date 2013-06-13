package remoterobot;

import java.awt.*;
import java.awt.event.*;
import java.awt.geom.Point2D;
import java.io.IOException;
import static java.lang.Thread.sleep;
import java.util.LinkedList;
import java.util.Vector;
import java.util.logging.Level;
import java.util.logging.Logger;
import javax.swing.*;

import slam.EKFSLAM;
import utils.Point;
import utils.Angle;
import utils.Pose;

import lejos.nxt.Motor;
import lejos.nxt.SensorPort;
import lejos.nxt.remote.NXTCommand;
import lejos.pc.comm.NXTComm;
import lejos.pc.comm.NXTCommLogListener;
import lejos.pc.comm.NXTCommandConnector;
import lejos.pc.comm.NXTConnector;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.util.Matrix;
import lejos.nxt.UltrasonicSensor;

public class RemoteRobot extends JFrame {

	DifferentialPilot pilot = new DifferentialPilot(4, 14f, Motor.A,
			Motor.B, false);

	// static variables
	public static final int CLOSURE = 60;
	public static int FORWARD_SPEED = 350;
	public static int ANGLE_SPEED = 200;
	public static final int STOP = 0;
	public static final int FORWARD = 1;
	public static final int BACKWARD = 2;
	public static final int LEFT = 3;
	public static final int RIGHT = 4;
	public static int direction = STOP;
	public static boolean IS_LEFT, IS_RIGHT, IS_FORWARD, IS_BACKWARD;
	public static NXTConnector conn;
	public static UltrasonicSensor sonic;
	public static EKFSLAM ekf;
	public static MapGUI mapTest;
	private static RemoteRobot instance = null;

	public static RemoteRobot getInstance() {
		if (instance == null)
			instance = new RemoteRobot();
		return instance;
	}

	public static JButton quit, connect;
	public static ButtonHandler bh = new ButtonHandler();

	public RemoteRobot() {
		setTitle("Control");
		setBounds(650, 350, 200, 100);
		setLayout(new GridLayout(2, 1));
		Motor.A.setSpeed(400);
		Motor.B.setSpeed(400);
		connect = new JButton(" Connect ");
		connect.addActionListener(bh);
		connect.addKeyListener(bh);
		add(connect);

		// Covariance of motion error
		double rt[][] = { { 0.01209283, -0.01883810, -0.02272150 },
				{ -0.01883810, 0.03107288, 0.03688513 },
				{ -0.02272150, 0.03688513, 0.04397788 } };
		// Covariance of sensor error
		double qt[][] = { { 0.2, 0, 0 }, { 0, 0.3, 0 }, { 0, 0, 0 }, };
		Matrix Rt = new Matrix(rt);
		Matrix Qt = new Matrix(qt);
		ekf = new EKFSLAM(new Pose(0, 0, 0), Rt, Qt, 2);

		sonic = new UltrasonicSensor(SensorPort.S4);
		sonic.capture();
		quit = new JButton("Quit");
		quit.addActionListener(bh);
		add(quit);

	}

	// bluetooth connection to NXT called "idefix"
	public static void connect() {
		conn = new NXTConnector();
		conn.addLogListener(new NXTCommLogListener() {
			public void logEvent(String message) {
				System.out.println(message);
			}

			public void logEvent(Throwable throwable) {
				System.err.println(throwable.getMessage());
			}
		});
		conn.setDebug(true);
		if (!conn.connectTo("btspp://brygada", NXTComm.LCP)) {
			System.err.println("Failed to connect");
			System.exit(1);
		}
		NXTCommandConnector.setNXTCommand(new NXTCommand(conn.getNXTComm()));

		initialize();
		//
	}

	public static void disconnect() {
		try {
			conn.close();
			System.exit(0);
		} catch (IOException ex) {
			Logger.getLogger(RemoteRobot.class.getName()).log(Level.SEVERE,
					null, ex);
		}

	}

	// set speeds, initialize sensors and starts thread that reads data from
	// sensors
	public static void initialize() {
	}

	private static class ButtonHandler implements ActionListener, KeyListener {
		// ***********************************************************************
		// Buttons action

		public void actionPerformed(ActionEvent ae) {
			System.out.println("Click");
			if (ae.getSource() == quit) {
				disconnect();
			}
			if (ae.getSource() == connect) {
				System.out.println("Connect");
				connect();
			}

		}// End ActionEvent(for buttons)

		// ***********************************************************************
		// Keyboard action
		float lastAngleIncrement = 0;
		float lastCount = 0;
		int threadA = Motor.A.getTachoCount();
		int threadB = Motor.B.getTachoCount();

		public void keyPressed(KeyEvent ke) {
			if (ke.getKeyChar() == 'q' && direction != STOP) {
				int dA = Motor.A.getTachoCount() - threadA;
				int dB = Motor.B.getTachoCount() - threadB;

				Motor.A.stop();
				Motor.B.stop();

				while (RemoteRobot.getInstance().pilot.isMoving()) {
					//
				}
				
				double dist = sonic.getDistance();
				
				if (dist < 200) {
//					util.Point2D p = new util.Point2D(dist, 0);
//					Vector<util.Point2D> v = new Vector<util.Point2D>();
//					v.add(p);
//					Alg_SEIF_SLAM_known_correspondences.getInstance()
//							.setMeasurement(v);
				}
				
				Motor.C.resetTachoCount();
				Motor.C.setSpeed(100);
				double movementNormalizer = 0.036;
				double angleNormalizer = 1 / 0.08;
				double movement = ((dB + dA) / 2);

				movement *= movementNormalizer;
				System.out.println("Movement " + String.valueOf(movement));
				
				double angle = ((dB - dA) / 2) / (14 * Math.PI);
				angle *= angleNormalizer;

				System.out.println("Angle " + String.valueOf(angle));
				
				if (direction == FORWARD || direction == BACKWARD) {
					angle = 0;
				}
				if (direction == LEFT || direction == RIGHT) {
					movement = 0;
				}
				
				double dr = movement;
				double x = mapTest.robot.pos.getX();
				double y = mapTest.robot.pos.getY();
				double t = mapTest.robot.rot;
				double dx = x + Math.cos(t);
				double dy = y + Math.sin(t);
				
				System.out.println(String.valueOf(dx) + " "
						+ String.valueOf(dy));
				if (direction == FORWARD) {
					mapTest.moveRobot(dr);
				}
				if (direction == BACKWARD) {
					mapTest.moveRobot(dr);
				}
				if (direction == LEFT) {
					mapTest.rotateRobot(angle);
				}
				if (direction == RIGHT) {
					mapTest.rotateRobot(angle);
				}

				ekf.motionUpdate(new Point((float) dx, (float) dy), new Angle(0));
				
				direction = STOP;

				threadA = Motor.A.getTachoCount();
				threadB = Motor.B.getTachoCount();
			}

			if (ke.getKeyChar() == 'w' && direction == STOP) {
				RemoteRobot.getInstance().pilot.forward();
				Motor.A.setSpeed(FORWARD_SPEED);
				Motor.B.setSpeed(FORWARD_SPEED);
				Motor.A.forward();
				Motor.B.forward();
				direction = FORWARD;
			}
			if (ke.getKeyChar() == 's' && direction == STOP) {
				direction = BACKWARD;
				Motor.A.setSpeed(FORWARD_SPEED);
				Motor.B.setSpeed(FORWARD_SPEED);
				Motor.A.backward();
				Motor.B.backward();

			}
			if (ke.getKeyChar() == 'a' && direction == STOP) {
				direction = LEFT;
				Motor.A.setSpeed(ANGLE_SPEED);
				Motor.B.setSpeed(ANGLE_SPEED);
				Motor.B.backward();
				Motor.A.forward();

			}
			if (ke.getKeyChar() == 'd' && direction == STOP) {
				Motor.A.setSpeed(ANGLE_SPEED);
				Motor.B.setSpeed(ANGLE_SPEED);
				Motor.A.backward();
				Motor.B.forward();
				direction = RIGHT;
			}

		}// End keyPressed

		@Override
		public void keyReleased(KeyEvent arg0) {

		}

		@Override
		public void keyTyped(KeyEvent arg0) {

		}
	}// End ButtonHandler

	public static void main(String[] args) throws Exception {
		RemoteRobot NXTrc = RemoteRobot.getInstance();
		NXTrc.setVisible(true);
		NXTrc.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

		mapTest = new MapGUI();
		mapTest.run();
		mapTest.createBufferStrategy(2);

	}
}