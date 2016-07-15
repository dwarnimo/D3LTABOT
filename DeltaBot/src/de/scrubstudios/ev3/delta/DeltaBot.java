package de.scrubstudios.ev3.delta;

import java.rmi.RemoteException;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import de.scrubstudios.ev3.SimpleTouch;
import de.scrubstudios.ev3.Angle3;
import de.scrubstudios.ev3.Point3;
import lejos.hardware.Brick;
import lejos.hardware.BrickFinder;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.sensor.EV3IRSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

@SuppressWarnings("unused")
public class DeltaBot {

	// ROBOT GEOMETRY
	// ============================================================================================

	// radius from base center to shoulder joint
	public static final float BASE = 66.4f;

	// radius from effector center to wrist joint
	public static final float EE = 30;

	// length of upper arm (length from shoulder joint to elbow joint)
	public static final int BICEP = 80;

	// length of lower arm (length from elbow joint to wrist joint)
	public static final int FOREARM = 208;

	// number of motor revolutions required for one rotation of shoulder joint
	public static final int GEAR_RATIO = -24;

	// TRIGONOMETRIC CONSTANTS
	// ============================================================================================

	private static final float PI = 3.141592f;
	private static final float SQRT3 = (float) Math.sqrt(3);
	private static final float SIN120 = SQRT3 / 2;
	private static final float COS120 = -0.5f;
	private static final float TAN60 = SQRT3;
	private static final float SIN30 = 0.5f;
	private static final float TAN30 = 1 / SQRT3;
	private static final float D2R = PI / 180; // deg to rad
	private static final float R2D = 180 / PI; // rad to deg

	// EV3 COMPONENTS
	// ============================================================================================

	private Brick ev3;
	private RegulatedMotor[] motors = new RegulatedMotor[3];
	private RegulatedMotor poti;
	private EV3IRSensor ir;
	private EV3TouchSensor touch;
	private SampleProvider samp;
	private SimpleTouch touch1;

	private boolean isValidPos = true;

	// CONSTRUCTOR
	// ============================================================================================

	public DeltaBot() {

		init();
		calibrate();
	}

	/*
	 * initialize ev3 components
	 */
	private void init() {
		ev3 = BrickFinder.getLocal();
		motors[0] = new EV3LargeRegulatedMotor(ev3.getPort("A"));
		motors[1] = new EV3LargeRegulatedMotor(ev3.getPort("B"));
		motors[2] = new EV3LargeRegulatedMotor(ev3.getPort("C"));
		poti = new EV3MediumRegulatedMotor(ev3.getPort("D"));
		ir = new EV3IRSensor(ev3.getPort("S1"));
		touch = new EV3TouchSensor(ev3.getPort("S2"));
		samp = touch.getTouchMode();
		touch1 = new SimpleTouch(samp);
	}

	/*
	 * manual calibration sequence. rotate wheel until the arm is in 0 position
	 * (horizontal) and the press the button to set the new 0 position. repeat
	 * for each arm.
	 */
	public void calibrate() {
		for (RegulatedMotor m : motors) {
			int theta;
			while (!touch1.isPressed()) {
				theta = poti.getTachoCount();
				LCD.drawString("Theta: " + theta, 1, 1);
				m.rotateTo(theta, true);
				Delay.msDelay(50);
				LCD.clear();
			}
			m.resetTachoCount();
			poti.resetTachoCount();
			LCD.clear();
			Sound.beep();
			while (touch1.isPressed()) { // ensures button must be released to continue to next arm
				Delay.msDelay(50);
			}
		}
		Delay.msDelay(500);
		Sound.beepSequenceUp();
	}

	// SETTERS & GETTERS
	// ============================================================================================

	/*
	 * set motor speed for all motors
	 */
	public void setMotorSpeed(int speed) {
		for (RegulatedMotor m : motors) {
			m.setSpeed(speed);
		}
	}

	/*
	 * set desired acceleration for all motors
	 */
	public void setMotorAcc(int acc) {
		for (RegulatedMotor m : motors) {
			m.setAcceleration(acc);
		}
	}

	/*
	 * set and move to the desired MOTOR angles
	 */
	public void setMotorAngles(Angle3 ang) {
		motors[0].rotateTo((int) ang.t1, true);
		motors[1].rotateTo((int) ang.t2, true);
		motors[2].rotateTo((int) ang.t3, true);
	}

	public void setMotorAngles(int t1, int t2, int t3) {
		int[] ang = new int[] { t1, t2, t3 };
		for (int i = 0; i < motors.length; ++i) {
			motors[i].rotateTo(ang[i], true);
		}
	}

	/*
	 * set and move to the desired JOINT angles
	 */
	public void setJointAngles(Angle3 ang) {
		motors[0].rotateTo((int) ang.t1 * GEAR_RATIO, true);
		motors[1].rotateTo((int) ang.t2 * GEAR_RATIO, true);
		motors[2].rotateTo((int) ang.t3 * GEAR_RATIO, true);
	}

	public void setJointAngles(int t1, int t2, int t3) {
		int[] ang = new int[] { t1, t2, t3 };
		for (int i = 0; i < motors.length; ++i) {
			motors[i].rotateTo(ang[i] * GEAR_RATIO, true);
		}
	}

	/*
	 * get the current angles of the shoulder joints
	 */
	public Angle3 getJointAngles() {
		return new Angle3(motors[0].getTachoCount() / GEAR_RATIO, 
						  motors[1].getTachoCount() / GEAR_RATIO,
						  motors[2].getTachoCount() / GEAR_RATIO);
	}

	/*
	 * get the actual motor angles for the current joint angles
	 */
	public Angle3 getMotorAngles() {
		return new Angle3(motors[0].getTachoCount(), 
						  motors[1].getTachoCount(), 
						  motors[2].getTachoCount());
	}

	/*
	 * get the current end-effector position
	 */
	public Point3 getCurrentPos() {
		return calcFK(getJointAngles());
	}

	// MISC METHODS
	// ============================================================================================

	/*
	 * wait until all motors have stopped moving
	 */
	public void motorsWaitComplete() {
		while (motors[0].isMoving() || motors[1].isMoving() || motors[2].isMoving()) {
			Delay.msDelay(10);
		}
		ev3.getAudio().systemSound(0);
	}

	/*
	 * close all open ports
	 */
	public void closePorts() {
		for (RegulatedMotor m : motors) {
			m.close();
		}
		ir.close();
		touch.close();
	}

	// INVERSE KINEMATICS
	// ============================================================================================

	/*
	 * helper function to generate the joint angle for the YZ-plane for the
	 * specified position
	 */
	private float calcAngYZ(Point3 pos) {
		float theta = 0;
		float L3 = (float) Math.sqrt(FOREARM * FOREARM - pos.x * pos.x);
		float Ay = -BASE;
		float PCy = pos.y - EE;
		float dn = (float) (BICEP + Math.sqrt(FOREARM * FOREARM - (Ay - PCy) * (Ay - PCy)));

		if (Math.abs(pos.z) <= dn) {

			float a = (4 * pos.z * pos.z + (2 * Ay - 2 * PCy) * (2 * Ay - 2 * PCy));
			
			float b = (-8 * Ay * pos.z * pos.z + (4 * Ay - 4 * PCy) * (BICEP * BICEP 
					- L3 * L3 - Ay * Ay + PCy * PCy + pos.z * pos.z));

			float c = (4 * Ay * Ay * pos.z * pos.z - (4 * pos.z * pos.z) * (BICEP * BICEP) 
					+ (BICEP * BICEP - L3 * L3 - Ay * Ay + PCy * PCy + pos.z * pos.z) 
					* (BICEP * BICEP - L3 * L3 - Ay * Ay + PCy * PCy + pos.z * pos.z));

			float D = (float) Math.sqrt(b * b - 4 * a * c);

			float By1 = (-b - D) / (2 * a);
			float By2 = (-b + D) / (2 * a);

			if (Math.abs(By1) > Math.abs(By2)) {
				float Bz1 = (By1 * (2 * Ay - 2 * PCy) + BICEP * BICEP 
						  - L3 * L3 - Ay * Ay + PCy * PCy + pos.z * pos.z) / (2 * pos.z);
				
				theta = (float) (Math.atan(Bz1 / (Ay - By1)) * R2D);
			} else {
				float Bz1 = (By2 * (2 * Ay - 2 * PCy) + BICEP * BICEP 
						  - L3 * L3 - Ay * Ay + PCy * PCy + pos.z * pos.z) / (2 * pos.z);
				
				theta = (float) (Math.atan(Bz1 / (Ay - By2)) * R2D);
			}
			isValidPos = true;
			return theta;
		} else {
			isValidPos = false;
		}
		return 0;
	}

	/*
	 * calculate the joint angles t1, t2, t3 for the specified position.
	 * for t2 and t3 the word coordinate system is rotated by 120 and 240
	 * degrees respectively.
	 */
	public Angle3 calcIK(Point3 pos) {
		float t1 = calcAngYZ(pos);
		float t2 = calcAngYZ(pos.rotZ(120 * D2R));
		float t3 = calcAngYZ(pos.rotZ(240 * D2R));												
		return new Angle3(t1, t2, t3);
	}

	/*
	 * calculate the joint angles t1, t2, t3 for the specified coordinates x, y, z
	 */
	public Angle3 calcIK(float x, float y, float z) {
		return calcIK(new Point3(x, y, z));
	}

	// FORWARD KINEMATICS
	// ============================================================================================

	/*
	 * calculate the end-effector position for the specified joint angles t1,
	 * t2, t3
	 */
	public Point3 calcFK(Angle3 ang) {
		ang.t1 *= D2R;
		ang.t2 *= D2R;
		ang.t3 *= D2R;

		float y1 = (float) -(BASE - EE + BICEP * Math.cos(ang.t1));
		float z1 = (float) (-BICEP * Math.sin(ang.t1));

		float y2 = (float) ((BASE - EE + BICEP * Math.cos(ang.t2)) * SIN30);
		float x2 = y2 * TAN60;
		float z2 = (float) (-BICEP * Math.sin(ang.t2));

		float y3 = (float) ((BASE - EE + BICEP * Math.cos(ang.t3)) * SIN30);
		float x3 = -y3 * TAN60;
		float z3 = (float) (-BICEP * Math.sin(ang.t3));

		float dnm = (y2 - y1) * x3 - (y3 - y1) * x2;

		float w1 = y1 * y1 + z1 * z1;
		float w2 = x2 * x2 + y2 * y2 + z2 * z2;
		float w3 = x3 * x3 + y3 * y3 + z3 * z3;

		// x = (a1*z + b1);
		float a1 = ((z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1)) / dnm;
		float b1 = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) / (2 * dnm);

		// y = (a2*z + b2);
		float a2 = -((z2 - z1) * x3 - (z3 - z1) * x2) / dnm;
		float b2 = ((w2 - w1) * x3 - (w3 - w1) * x2) / (2 * dnm);

		// a*z^2 + b*z + c = 0
		float a = a1 * a1 + a2 * a2 + 1;
		float b = 2 * (a1 * b1 + a2 * (b2 - y1) - z1);
		float c = b1 * b1 + (b2 - y1) * (b2 - y1) + z1 * z1 - FOREARM * FOREARM;

		// discriminant
		float d = b * b - 4 * a * c;
		if (d < 0) {
			// TODO
		}

		float z0 = (float) (-(-b + Math.sqrt(d)) / (2 * a));
		float x0 = (-a1 * z0 + b1);
		float y0 = (-a2 * z0 + b2);

		return new Point3(x0, y0, z0);
	}

	/*
	 * calculate the end-effector position for the specified joint angles t1,
	 * t2, t3
	 */
	public Point3 calcFK(float t1, float t2, float t3) {
		return calcFK(new Angle3(t1, t2, t3));
	}

	// PATH GENERATION
	// ============================================================================================

	/*
	 * calculate points between current and desired position using linear
	 * interpolation. a lower minimum distance means more intermediate points
	 * are generated.
	 */
	private List<Point3> interp(Point3 posd) {
		Point3 pos0 = new Point3(getCurrentPos());
		float minDist = 2;
		float dist = pos0.getDist(posd);
		float steps = (float) Math.ceil(dist / minDist);
		Point3 inc = new Point3((posd.x - pos0.x) / steps,
								(posd.y - pos0.y) / steps,
								(posd.z - pos0.z) / steps);
		
		List<Point3> points = new ArrayList<>();
		for (int i = 1; i < steps; ++i) {
			points.add(new Point3(pos0.x + i * inc.x,
								  pos0.y + i * inc.y,
								  pos0.z + i * inc.z));
		}
		return points;
	}

	/*
	 * iterate through the list of points and calculate the joint angles for
	 * each point
	 */
	private List<Angle3> calcAngles(List<Point3> points) {
		List<Angle3> angles = new ArrayList<>();
		for (Iterator<Point3> iter = points.iterator(); iter.hasNext();) {
			Point3 currentPoint = iter.next();
			angles.add(calcIK(currentPoint));
		}
		return angles;
	}

	// MOVEMENT METHODS
	// ============================================================================================

	/*
	 * move smoothly to the desired position in a straight line. intermediate
	 * points are calculated using linear interpolation
	 */
	public void moveToPosLin(Point3 posd, int minDist) {
		List<Angle3> angles = new ArrayList<>();
		angles = calcAngles(interp(posd));
		Iterator<Angle3> iter = angles.iterator();
		while (iter.hasNext()) {
			Angle3 currentAngles = iter.next();
			setJointAngles(currentAngles);
			Delay.msDelay(25);
		}
		motorsWaitComplete();
	}

	/*
	 * move directly to the desired position without using intermediate points
	 */
	public void moveToPosDirect(Point3 pos) {
		setJointAngles(calcIK(pos));
	}

	/*
	 * move robot to home position
	 */
	public void moveHome() {
		setMotorAngles(0, 0, 0);
	}

	// MAIN METHOD
	// ============================================================================================

	public static void main(String[] args) {
		DeltaBot delta = new DeltaBot();

		delta.setMotorSpeed(800);
		delta.setMotorAcc(1000);

		delta.moveToPosLin(new Point3(0, 0, -200), 1);
		delta.motorsWaitComplete();
		Delay.msDelay(2000);

		delta.moveToPosLin(new Point3(-50, 0, -200), 1);
		delta.motorsWaitComplete();
		Delay.msDelay(2000);

		delta.moveToPosLin(new Point3(50, 0, -180), 1);
		delta.motorsWaitComplete();
		Delay.msDelay(2000);

		delta.moveHome();
		delta.motorsWaitComplete();

		delta.closePorts();
	}
}
