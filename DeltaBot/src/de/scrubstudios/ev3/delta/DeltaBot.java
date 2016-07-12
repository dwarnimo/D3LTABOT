package de.scrubstudios.ev3.delta;

import java.rmi.RemoteException;

import de.scrubstudios.ev3.SimpleTouch;
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

	// ROBOT GEOMETRY =============================================================================

	// radius from base center to shoulder joint
	public static final float BASE = 66.4f;

	// radius from effector center to wrist joint
	public static final float EE = 30;

	// length of upper arm (length from shoulder joint to elbow joint)
	public static final int BICEP = 80;

	// length of lower arm (length from elbow joint to wrist joint)
	public static final int FOREARM = 200;

	// length of upper arm squared
	public static final int BICEP_SQ = BICEP * BICEP;

	// length of lower arm squared
	public static final int FOREARM_SQ = FOREARM * FOREARM;

	// number of motor revolutions required for one rotation of shoulder joint
	public static final int GEAR_RATIO = 1; // FIXME

	// TRIGONOMETRIC CONSTANTS ====================================================================

	private static final float PI = 3.141592f;
	private static final float SQRT3 = (float) Math.sqrt( 3 );
	private static final float SIN120 = SQRT3 / 2;
	private static final float COS120 = -0.5f;
	private static final float TAN60 = SQRT3;
	private static final float SIN30 = 0.5f;
	private static final float TAN30 = 1 / SQRT3;
	private static final float D2R = PI / 180; // deg to rad
	private static final float R2D = 180 / PI; // rad to deg

	private boolean isValidPos = true;

	private Brick ev3;

	private RegulatedMotor[] motors = new RegulatedMotor[3];
	private RegulatedMotor poti;

	private EV3IRSensor ir;
	private EV3TouchSensor touch;
	private SampleProvider samp;
	private SimpleTouch touch1;

	public DeltaBot() {

		init();
		calibrate();
	}

	private void init() {

		ev3 = BrickFinder.getLocal();

		motors[0] = new EV3LargeRegulatedMotor( ev3.getPort( "A" ) );
		motors[1] = new EV3LargeRegulatedMotor( ev3.getPort( "B" ) );
		motors[2] = new EV3LargeRegulatedMotor( ev3.getPort( "C" ) );
		poti = new EV3MediumRegulatedMotor( ev3.getPort( "D" ) );

		ir = new EV3IRSensor( ev3.getPort( "S1" ) );
		touch = new EV3TouchSensor( ev3.getPort( "S2" ) );
		samp = touch.getTouchMode();
		touch1 = new SimpleTouch( samp );

	}

	public void calibrate() {

		for ( RegulatedMotor m : motors ) {
			int theta;
			while ( !touch1.isPressed() ) {
				theta = poti.getTachoCount();
				LCD.drawString( "Theta: " + theta, 1, 1 );
				m.rotateTo( theta, true );
				Delay.msDelay( 50 );
				LCD.clear();
			}
			m.resetTachoCount();
			poti.resetTachoCount();
			LCD.clear();
			Sound.beep();
			while ( touch1.isPressed() ) {
				Delay.msDelay( 50 );
			}
		}
		Delay.msDelay( 500 );
		Sound.beepSequenceUp();
	}

	// SETTERS & GETTERS ==========================================================================

	public void setMotorSpeed(int speed) {

		for ( RegulatedMotor m : motors ) {
			m.setSpeed( speed );
		}
	}

	public void setMotorAcc(int acc) {

		for ( RegulatedMotor m : motors ) {
			m.setAcceleration( acc );
		}
	}

	public void setMotorAngles(int a1, int a2, int a3) {

		int[] angles = new int[] { a1, a2, a3 };
		for ( int i = 0; i < angles.length; i++ ) {
			motors[i].rotateTo( angles[i], true );
		}
	}

	public void setMotorAngles(int[] angles) {

		for ( int i = 0; i < motors.length; ++i ) {
			motors[i].rotateTo( angles[i], true );
		}
	}

	public void setJointAngles(int a1, int a2, int a3) {

		int[] angles = new int[] { a1, a2, a3 };
		for ( int i = 0; i < angles.length; i++ ) {
			motors[i].rotateTo( angles[i] * GEAR_RATIO, true );
		}
	}

	public void setJointAngles(int[] angles) {

		for ( int i = 0; i < motors.length; ++i ) {
			motors[i].rotateTo( angles[i] * GEAR_RATIO, true );
		}
	}

	public int[] getJointAngles() {

		int[] angles = new int[3];
		for ( int i = 0; i < motors.length; ++i ) {
			angles[i] = motors[i].getTachoCount() / GEAR_RATIO;
		}
		return angles;
	}

	public int[] getMotorAngles() {

		int[] angles = new int[3];
		for ( int i = 0; i < motors.length; ++i ) {
			angles[i] = motors[i].getTachoCount();
		}
		return angles;
	}

	public int[] getCurrentPos() {

		return calcFK( getJointAngles() );
	}

	// MISC METHODS ===============================================================================

	public void motorsWaitComplete() {

		while ( motors[0].isMoving() || motors[1].isMoving() || motors[2].isMoving() ) {
			Delay.msDelay( 10 );
		}
		ev3.getAudio().systemSound( 0 );
	}

	public void closePorts() {

		for ( RegulatedMotor m : motors ) {
			m.close();
		}
		ir.close();
		touch.close();
	}

	public float getDist(int[] p1, int[] p2) {

		float dist = (float) Math.sqrt( (p2[0] - p1[0]) * (p2[0] - p1[0]) + (p2[1] - p1[1]) * (p2[1] - p1[1])
				+ (p2[2] - p1[2]) * (p2[2] - p1[2]) );
		return dist;
	}

	// INVERSE KINEMATICS =========================================================================

	private int CalcAngYZ(float x0, float y0, float z0) {

		y0 += EE - BASE; //
		float A = (x0 * x0 + y0 * y0 + z0 * z0 + BICEP_SQ - FOREARM_SQ) / (2 * z0);
		float B = y0 / z0;
		float a = B * B + 1;
		float b = -2 * A * B;
		float c = A * A - BICEP_SQ;
		float D = b * b - 4 * a * c;
		if ( D < 0 ) {
			isValidPos = false;
			return 0;
		}
		float y = (float) ((-b + Math.sqrt( D )) / (2 * a));
		float z = A - y * B;
		return (int) Math.rint( (Math.atan2( z, y ) * R2D * GEAR_RATIO) );
	}

	public int[] calcIK(float[] pos0) {

		int[] angles = new int[3];
		angles[0] = CalcAngYZ( pos0[0], pos0[1], pos0[2] );
		
		angles[1] = CalcAngYZ( pos0[0] * COS120 + pos0[1] * SIN120,
							   pos0[1] * COS120 - pos0[0] * SIN120, 
							   pos0[2] );
		
		angles[2] = CalcAngYZ( pos0[0] * COS120 - pos0[1] * SIN120,
							   pos0[1] * COS120 + pos0[0] * SIN120,
							   pos0[2] );

		return angles;
	}

	public int[] calcIK(float x0, float y0, float z0) {

		float[] pos0 = new float[] { x0, y0, z0 };
		return calcIK( pos0 );
	}

	// FORWARD KINEMATICS =========================================================================

	public int[] calcFK(int[] thetas) {

		int[] pos = new int[3];

		thetas[0] *= D2R;
		thetas[1] *= D2R;
		thetas[2] *= D2R;

		float y1 = (float) -(BASE - EE + BICEP * Math.cos( thetas[0] ));
		float z1 = (float) (-BICEP * Math.sin( thetas[0] ));

		float y2 = (float) ((BASE - EE + BICEP * Math.cos( thetas[1] )) * SIN30);
		float x2 = y2 * TAN60;
		float z2 = (float) (-BICEP * Math.sin( thetas[1] ));

		float y3 = (float) ((BASE - EE + BICEP * Math.cos( thetas[2] )) * SIN30);
		float x3 = -y3 * TAN60;
		float z3 = (float) (-BICEP * Math.sin( thetas[2] ));

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
		float c = b1 * b1 + (b2 - y1) * (b2 - y1) + z1 * z1 - FOREARM_SQ;

		// discriminant
		float d = b * b - 4 * a * c;
		if ( d < 0 ) {
			// TODO
		}

		float z0 = (float) (-(-b + Math.sqrt( d )) / (2 * a));
		float x0 = (-a1 * z0 + b1);
		float y0 = (-a2 * z0 + b2);

		pos[0] = (int) Math.round( x0 );
		pos[1] = (int) Math.round( y0 );
		pos[2] = (int) Math.round( z0 );

		return pos;
	}

	public int[] calcFK(int t1, int t2, int t3) {

		int[] thetas = new int[] { t1, t2, t3 };
		return calcFK( thetas );
	}

	public void moveHome() {

		setMotorAngles( 0, 0, 0 );
	}

	public void moveToPos(int xd, int yd, int zd, int minDist) {

		int[] posd = new int[] { xd, yd, zd };
		int[] pos0 = new int[3];

		pos0 = getCurrentPos();

		float length = getDist( posd, pos0 );

		float[] inc = new float[3];
		for ( int i = 0; i < inc.length; ++i ) {
			inc[i] = (posd[i] - pos0[i]) / (length / minDist);
		}

		int[][] thetas = new int[(int) Math.floor( (length / minDist) + 2 )][3];

		for ( int i = 0; i < ((length / minDist) + 1); ++i ) {
			int[] thetas_ = new int[3];
			thetas_ = (calcIK( pos0[0] + i * inc[0], pos0[1] + i * inc[1], pos0[2] + i * inc[2] ));
			thetas[i][0] = thetas_[0] * GEAR_RATIO;
			thetas[i][1] = thetas_[1] * GEAR_RATIO;
			thetas[i][2] = thetas_[2] * GEAR_RATIO;
		}

		for ( int i = 0; i < thetas.length; ++i ) {
			// System.out.println( "Waypoint " + i + ":" + " t1 = " + thetas[i][0] + " t2 = " + thetas[i][1]
			// + " t3 = " + thetas[i][2] );

			setMotorAngles( thetas[i][0], thetas[i][1], thetas[i][2] );
			// Delay.msDelay( 50 );
		}
		while ( motors[0].isMoving() || motors[1].isMoving() || motors[2].isMoving() ) {

		}
		ev3.getLED().setPattern( 1 );
		ev3.getAudio().systemSound( 1 );
		Delay.msDelay( 250 );
		ev3.getLED().setPattern( 0 );

	}

	public static void main(String[] args) {

		DeltaBot delta = new DeltaBot();

		int[] angles = new int[3];
		angles = delta.getJointAngles();
		for ( int i = 0; i < angles.length; ++i ) {
			System.out.println( angles[i] );
		}

		int[] pos = new int[3];
		pos = delta.getCurrentPos();
		for ( int i = 0; i < pos.length; ++i ) {
			System.out.println( pos[i] );
		}
	}
}
