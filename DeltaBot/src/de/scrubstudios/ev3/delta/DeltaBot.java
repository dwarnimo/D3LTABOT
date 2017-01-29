package de.scrubstudios.ev3.delta;

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

//@SuppressWarnings("unused")
public class DeltaBot {

	// ROBOT GEOMETRY
	// ============================================================================================

	// radius from base center to shoulder joint
	public final float base;

	// radius from effector center to wrist joint
	public final float ee;

	// length of bicep arm (length from shoulder joint to elbow joint)
	public final int bicep;

	// length of forearm arm (length from elbow joint to wrist joint)
	public final int forearm;

	// number of motor revolutions required for one rotation of shoulder joint
	public final int gearRatio;

	// TRIGONOMETRIC CONSTANTS ====================================================================

	private static final float PI = 3.141592f;
	private static final float SQRT3 = (float) Math.sqrt(3);
	private static final float SIN120 = SQRT3 / 2;
	private static final float COS120 = -0.5f;
	private static final float TAN60 = SQRT3;
	private static final float SIN30 = 0.5f;
	private static final float TAN30 = 1 / SQRT3;
	private static final float D2R = PI / 180; // deg to rad
	private static final float R2D = 180 / PI; // rad to deg

	// EV3 COMPONENTS =============================================================================

	private Brick ev3;
	private RegulatedMotor[] motors = new RegulatedMotor[3];
	private RegulatedMotor poti;
	private EV3IRSensor ir;
	private EV3TouchSensor touch;
	private SampleProvider samp;
	private SimpleTouch touch1;

	private boolean isValidPos = true;

	// CONSTRUCTOR ================================================================================

	public DeltaBot(float base, float ee, int upper, int lower , int gearRatio) {

		this.base = base;
		this.ee = ee;
		this.bicep = upper;
		this.forearm = lower;
		this.gearRatio = gearRatio;

		init();
		calibrate();
	}


	// initialize ev3 components
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
	 * (horizontal) and then press the button to confirm. repeat for each arm.
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

	// SETTERS & GETTERS ==========================================================================


	// set the same speed for all motors
	public void setSameMotorSpeedAll(int spd) {

		for (RegulatedMotor m : motors) {
			m.setSpeed(spd);
		}
	}

	// set individual speeds for all motors
	public void setMotorSpeedAll(int spdM1, int spdM2, int spdM3) {

		motors[0].setSpeed(spdM1);
		motors[1].setSpeed(spdM2);
		motors[2].setSpeed(spdM3);
	}

	// set motor speed for specified motor
	public void setMotorSpeed(int motor, int spd) {

		motors[motor].setSpeed(spd);
	}

	// set the same acceleration for all motors
	public void setSameMotorAccAll(int acc) {

		for (RegulatedMotor m : motors) {
			m.setAcceleration(acc);
		}
	}

	// set individual accelerations for all motors
	public void setMotorAccAll(int accM1, int accM2, int accM3) {

		motors[0].setAcceleration(accM1);
		motors[1].setAcceleration(accM2);
		motors[2].setAcceleration(accM3);
	}

	// set motor acceleration for specified motor
	public void setMotorAcc(int motor, int acc) {

		motors[motor].setAcceleration(acc);
	}

	// set all MOTOR angles to the desired values
	public void setMotorAnglesAll(Angle3 ang) {

		motors[0].rotateTo((int) ang.t1, true);
		motors[1].rotateTo((int) ang.t2, true);
		motors[2].rotateTo((int) ang.t3, true);
	}

	public void setMotorAnglesAll(int t1, int t2, int t3) {

		int[] ang = new int[] { t1, t2, t3 };
		for (int i = 0; i < motors.length; ++i) {
			motors[i].rotateTo(ang[i], true);
		}
	}

	// set all JOINT angles to the desired values
	public void setJointAnglesAll(Angle3 ang) {

		motors[0].rotateTo((int) ang.t1 * gearRatio, true);
		motors[1].rotateTo((int) ang.t2 * gearRatio, true);
		motors[2].rotateTo((int) ang.t3 * gearRatio, true);
	}

	public void setJointAnglesAll(int t1, int t2, int t3) {

		int[] ang = new int[] { t1, t2, t3 };
		for (int i = 0; i < motors.length; ++i) {
			motors[i].rotateTo(ang[i] * gearRatio, true);
		}
	}

	// returns the actual motor angles for the current joint angles
	public Angle3 getMotorAnglesAll() {

		return new Angle3(motors[0].getTachoCount(), 
				motors[1].getTachoCount(), 
				motors[2].getTachoCount());
	}

	// returns the current angles of all the shoulder joints
	public Angle3 getJointAnglesAll() {

		return new Angle3(motors[0].getTachoCount() / gearRatio, 
				motors[1].getTachoCount() / gearRatio,
				motors[2].getTachoCount() / gearRatio);
	}

	// returns the current end-effector position
	public Point3 getCurrentPos() {

		return calcFK(getJointAnglesAll());
	}


	// returns the angle of the specified joint
	public double getJointAngle(int joint) {

		return motors[joint].getTachoCount() / gearRatio;
	}

	// returns the actual motor angle of the specified joint
	public int getMotorAngle(int joint) {

		return motors[joint].getTachoCount();
	}

	// returns the motor target angle of the specified joint
	public int getJointLimitAngle(int joint) {

		return motors[joint].getLimitAngle();
	}

	// returns the error between joint target and actual position for the specified joint
	public int getJointDistance(int joint) {

		return motors[joint].getLimitAngle() - motors[joint].getTachoCount();
	}

	// returns true if the specified joint is moving
	public boolean isJointMoving(int joint) {

		return motors[joint].isMoving();
	}

	// returns true if at least one of the joints is moving
	public boolean areJointsMoving() {

		return motors[0].isMoving() || motors[1].isMoving() || motors[2].isMoving();
	}

	// wait for all motors to stop moving
	public void motorsWaitComplete() {

		while (motors[0].isMoving() || motors[1].isMoving() || motors[2].isMoving()) {
			Delay.msDelay(10);
		}
		ev3.getAudio().systemSound(0);
	}

	// close all open ports
	public void closePorts() {

		for (RegulatedMotor m : motors) {
			m.close();
		}
		ir.close();
		touch.close();
	}

	// INVERSE KINEMATICS =========================================================================

	/**
	 * Helper function to calculate the shoulder joint angle in the YZ-plane for the
	 * specified position.
	 * <p>
	 * <b>Point A</b> : Position of shoulder joint (actuated revolute joint between base plate and bicep)</br>
	 * <b>Point B</b> : Position of elbow joint (ball joint between bicep and forearm)</br>
	 * <b>Point C</b> : Position of wrist joint (ball joint between forarm and ee plate)</p>
	 * <p>
	 * <b>Ax, Bx</b> = 0 because we are operating strictly on the YZ-plane</br>
	 * <b>Ay</b> = base center shifted to edge (-base)</br>
	 * <b>Az</b> = 0 because the baseplate is located at the origin of the reference frame</p>
	 * <p>
	 * <b>By, Bz</b> : These are the coordinates we need to find in order to determine the shoudler joint angle</p>
	 * <p>
	 * <b>Cx</b> : Same as end-effector X-coordinate (pos.x)</br>
	 * <b>Cy</b> : End-effector Y-coordinate offset by end-effector radius (pos.y - ee)</br>
	 * <b>Cz</b> : Same as end-effector Z-coordinate (pos.z)</p>
	 * 
	 * @param pos The desired end-effector position
	 * @return Joint angle <b>theta</b> necessary to reach the desired end-effector position multiplied by the gear ratio
	 * @author dwarnimo
	 */
	private float calcAngYZ(Point3 pos) {

		float Ay = -base;
		float Cx = pos.x;
		float Cy = pos.y - ee;
		float Cz = pos.z;

		//projection of forearm onto YZ-plane
		float forearmYZ = (float) Math.sqrt(forearm * forearm - Cx * Cx);

		float dn = (float) (bicep + Math.sqrt(forearm * forearm - (Ay - Cy) * (Ay - Cy)));

		if (Math.abs(Cz) <= dn) {

			float helper = (bicep * bicep - forearmYZ * forearmYZ - Ay * Ay + Cy * Cy + Cz * Cz);

			float a = (4 * Cz * Cz + (2 * Ay - 2 * Cy) * (2 * Ay - 2 * Cy));
			float b = (-8 * Ay * Cz * Cz + (4 * Ay - 4 * Cy) * helper);
			float c = (4 * Ay * Ay * Cz * Cz - (4 * Cz * Cz) * (bicep * bicep) + helper * helper);

			float D = (float) Math.sqrt(b * b - 4 * a * c);

			float By1 = (-b - D) / (2 * a);
			float By2 = (-b + D) / (2 * a);

			float theta;

			if (Math.abs(By1) > Math.abs(By2)) {
				float Bz1 = (By1 * (2 * Ay - 2 * Cy) + bicep * bicep 
						- forearmYZ * forearmYZ - Ay * Ay + Cy * Cy + Cz * Cz) / (2 * Cz);

				theta = (float) (Math.atan(Bz1 / (Ay - By1)) * R2D * gearRatio);
			} else {
				float Bz1 = (By2 * (2 * Ay - 2 * Cy) + bicep * bicep 
						- forearmYZ * forearmYZ - Ay * Ay + Cy * Cy + Cz * Cz) / (2 * Cz);

				theta = (float) (Math.atan(Bz1 / (Ay - By2)) * R2D * gearRatio);
			}
			isValidPos = true;
			return theta;
		} else {
			isValidPos = false;
		}
		return 0;
	}

	/**
	 * Calculates the 3 motor angles required to reach the desired end-effector position
	 * by calling the <b>calcAngYZ</b> function for 3 different reference frames:</br>
	 * <p>
	 * <b>XYZ</b> : World Coordinate System (WCS)</br>
	 * <b>X'Y'Z'</b>: WCS rotated by 120 degrees CCW</br>
	 * <b>X"Y"Z"</b> : WCS rotated by 240 degrees CCW</br>
	 * </p>
	 * @param pos The desired end-effector position
	 * @return Joint angles <b>t1, t2, t3</b> multiplied by the gear ratio
	 */
	public Angle3 calcIK(Point3 pos) {

		float t1 = calcAngYZ(pos);
		float t2 = calcAngYZ(pos.rotZ(120 * D2R));
		float t3 = calcAngYZ(pos.rotZ(240 * D2R));												
		return new Angle3(t1, t2, t3);
	}

	/**
	 * Calculates the 3 motor angles required to reach the desired end-effector position
	 * by calling the <b>calcAngYZ</b> function for 3 different reference frames:</br>
	 * <p>
	 * <b>XYZ</b> : World Coordinate System (WCS)</br>
	 * <b>X'Y'Z'</b>: WCS rotated by 120 degrees CCW</br>
	 * <b>X"Y"Z"</b> : WCS rotated by 240 degrees CCW</br>
	 * </p>
	 * @param x The desired end-effector x-Coordinate
	 * @param y The desired end-effector y-Coordinate
	 * @param z The desired end-effector z-Coordinate
	 * @return Joint angles <b>t1, t2, t3</b> multiplied by the gear ratio
	 */
	public Angle3 calcIK(float x, float y, float z) {

		return calcIK(new Point3(x, y, z));
	}

	// FORWARD KINEMATICS =========================================================================

	/**
	 * Calculates the EE position for the specified joint angles.
	 * @param ang Angle3 containing the joint angles <b>theta1, 2, 3</b> from which to determine the EE position
	 * @return Point3 containing the <b>XYZ</b> coordinates of the calculated position
	 */
	public Point3 calcFK(Angle3 ang) {

		ang.t1 *= D2R;
		ang.t2 *= D2R;
		ang.t3 *= D2R;

		float y1 = (float) -(base - ee + bicep * Math.cos(ang.t1));
		float z1 = (float) (-bicep * Math.sin(ang.t1));

		float y2 = (float) ((base - ee + bicep * Math.cos(ang.t2)) * SIN30);
		float x2 = y2 * TAN60;
		float z2 = (float) (-bicep * Math.sin(ang.t2));

		float y3 = (float) ((base - ee + bicep * Math.cos(ang.t3)) * SIN30);
		float x3 = -y3 * TAN60;
		float z3 = (float) (-bicep * Math.sin(ang.t3));

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
		float c = b1 * b1 + (b2 - y1) * (b2 - y1) + z1 * z1 - forearm * forearm;

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

	/**
	 * Calculates the EE position for the specified joint angles.
	 * @param t1 Joint angle theta1
	 * @param t2 Joint angle theta2
	 * @param t3 Joint angle theta3
	 * @return Point3 containing the <b>XYZ</b> coordinates of the calculated position
	 */
	public Point3 calcFK(float t1, float t2, float t3) {
		return calcFK(new Angle3(t1, t2, t3));
	}

	// PATH GENERATION ============================================================================

	/**
	 * Calculates intermediate points between the current and the desired position using linear 
	 * interpolation.</br>
	 * A smaller <b>minDist</b> means that more intermediate points are generated.</br>
	 * @param posd The desired end-effector position
	 * @return <b>points</b>, A list containing the calculated intermediate points
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

	/**
	 * Iterates through the list of intermediate points and calculates the joint
	 * angles at each point.</br>
	 * These angles are stored in a new list.</br>
	 * 
	 * @param points
	 *            The list of interpolated points between the current and the
	 *            desired position
	 * @return <b>angles</b>, A list of path angles
	 */
	private List<Angle3> calcAngles(List<Point3> points) {

		List<Angle3> angles = new ArrayList<>();

		for (Iterator<Point3> iter = points.iterator(); iter.hasNext();) {
			Point3 currentPoint = iter.next();
			angles.add(calcIK(currentPoint));
		}
		return angles;
	}

	// MOVEMENT METHODS ===========================================================================

	/**
	 * Move to the desired position in a straight line.</br>
	 * Intermediate points are calculated using linear interpolation.</br>
	 * @param posd The desired end-effector position
	 */
	public void moveToPosLin(Point3 posd) {

		List<Angle3> angles = new ArrayList<>();
		angles = calcAngles(interp(posd));
		Iterator<Angle3> iter = angles.iterator();

		while (iter.hasNext()) {
			Angle3 currentAngles = iter.next();
			setMotorAnglesAll(currentAngles);

			int epsilon = 1; // max joint angle error 
			float currentTarget1, currentTarget2, currentTarget3;

			while ( true ) {
				currentTarget1 = currentAngles.t1 * gearRatio;
				currentTarget2 = currentAngles.t2 * gearRatio;
				currentTarget3 = currentAngles.t3 * gearRatio;

				if ( !areJointsMoving() )
					break;
				else {
					boolean needWait1 = isJointMoving(0) && ((getJointDistance(0) > 0 && currentTarget1 < getJointLimitAngle(0))
							|| (getJointDistance(0) < 0 && currentTarget1 > getJointLimitAngle(0)));

					boolean needWait2 = isJointMoving(1) && ((getJointDistance(1) > 0 && currentTarget2 < getJointLimitAngle(1))
							|| (getJointDistance(1) < 0 && currentTarget2 > getJointLimitAngle(1)));

					boolean needWait3 = isJointMoving(2) && ((getJointDistance(2) > 0 && currentTarget3 < getJointLimitAngle(2))
							|| (getJointDistance(2) < 0 && currentTarget3 > getJointLimitAngle(2)));

					if ( !needWait1 && !needWait2 && !needWait3) {
						// System.out.println("Starting early!");
						boolean startNext1 = !isJointMoving(0) || ((Math.abs( getJointDistance(0) ) <= epsilon));
						boolean startNext2 = !isJointMoving(1) || ((Math.abs( getJointDistance(1) ) <= epsilon));
						boolean startNext3 = !isJointMoving(2) || ((Math.abs( getJointDistance(2) ) <= epsilon));
						if ( startNext1 && startNext2 && startNext3)
							break;
					}
				}
			}	
		}
	}

	/**
	 * Move directly to the desired position without intermediate points.
	 * @param pos The desired end-effector position
	 */
	public void moveToPosDirect(Point3 pos) {

		setMotorAnglesAll(calcIK(pos));
	}

	/**
	 * Move arms to the home position. (<b>t1, t2, t3</b> = 0)
	 */
	public void moveHome() {

		setMotorAnglesAll(0, 0, 0);
	}

	// MAIN METHOD ================================================================================

	public static void main(String[] args) {

		DeltaBot delta = new DeltaBot(66.4f, 30, 80, 208, -24);

		delta.setSameMotorSpeedAll(800);
		delta.setSameMotorAccAll(3000);

		delta.moveToPosDirect(new Point3(0, 0, -200));
		delta.motorsWaitComplete();
		Delay.msDelay(500);

		delta.moveToPosDirect(new Point3(-80, 0, -200));
		delta.motorsWaitComplete();
		Delay.msDelay(500);

		delta.moveToPosDirect(new Point3(80, 0, -200));
		delta.motorsWaitComplete();
		Delay.msDelay(500);

		delta.moveToPosDirect(new Point3(0, 0, -230));
		delta.motorsWaitComplete();
		Delay.msDelay(500);

		delta.moveToPosDirect(new Point3(0, 0, -180));
		delta.motorsWaitComplete();
		Delay.msDelay(500);

		delta.moveToPosDirect(new Point3(30, 50, -230));
		delta.motorsWaitComplete();
		Delay.msDelay(500);

		delta.moveToPosDirect(new Point3(-30, -50, -230));
		delta.motorsWaitComplete();
		Delay.msDelay(500);

		delta.moveHome();
		delta.motorsWaitComplete();

		delta.closePorts();
	}
}
