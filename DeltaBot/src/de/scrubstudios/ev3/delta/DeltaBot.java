package de.scrubstudios.ev3.delta;

public class DeltaBot {

	DeltaKin kin;

	public DeltaBot() {

		kin = new DeltaKin();
	}

	public int[] getIK(float x0, float y0, float z0) {
		int[] angles = new int[3];
		angles = kin.calcIK( x0, y0, z0 );
		return angles;
	}

	public int[] getFK(int t1, int t2, int t3) {
		int[] pos = new int[3];
		pos = kin.calcFK( t1, t2, t3 );
		return pos;
	}

	public static void main(String[] args) {
		DeltaBot delta = new DeltaBot();

		int[] angles = new int[3];
		angles = delta.getIK( 0, 0, -200 );
		for ( int i = 0; i < angles.length; i++ ) {
			System.out.println( angles[i] );
		}

		int[] pos = new int[3];
		pos = delta.getFK( angles[0], angles[1], angles[2] );
		for ( int i = 0; i < pos.length; i++ ) {
			System.out.println( pos[i] );
		}

	}
}
