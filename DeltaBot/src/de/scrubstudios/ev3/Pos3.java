package de.scrubstudios.ev3;

public class Pos3 {

	public float x, y, z;
	
	public Pos3(){
		x = 0;
		y = 0;
		z = 0;
	}

	public Pos3(float[] pos) {
		x = pos[0];
		y = pos[1];
		z = pos[2];
	}

	public Pos3(float x, float y, float z) {
		this.x = x;
		this.y = y;
		this.z = z;
	}

	public Pos3(Pos3 pos) {
		x = pos.x;
		y = pos.y;
		z = pos.z;
	}

	public void setXYZ(float newX, float newY, float newZ) {

		x = newX;
		y = newY;
		z = newZ;
	}

	public void setXYZ(Pos3 newPos) {

		x = newPos.x;
		y = newPos.y;
		z = newPos.z;
	}

	public void setX(float newX) {

		x = newX;
	}

	public void setY(float newY) {

		y = newY;
	}

	public void setZ(float newZ) {

		z = newZ;
	}

	public Pos3 getXYZ() {

		Pos3 pos = new Pos3( x, y, z );
		return pos;
	};

	public float getX() {

		return x;
	}

	public float getY() {

		return y;
	}

	public float getZ() {

		return z;
	}

	public float getDistSquared(Pos3 pos2) {

		return (x - pos2.x) * (x - pos2.x) + (y - pos2.y) * (y - pos2.y) + (z - pos2.z) * (z - pos2.z);
	}

	public float getDist(Pos3 pos2) {

		return (float) Math.sqrt( getDistSquared( pos2 ) );
	}

	public Pos3 normDir(Pos3 pos2){
		
		float dist = getDist(pos2);
		Pos3 normDir = new Pos3();
		normDir.setX((pos2.x - x) / dist);
		normDir.setY((pos2.y - y) / dist);
		normDir.setZ((pos2.z - z) / dist);
		return normDir;	
	}
	
	public Pos3 rotZ(float ang) {

		float newX = (float) (x * Math.cos( ang ) + y * Math.sin( ang ));
		float newY = (float) (y * Math.cos( ang ) - x * Math.sin( ang ));
		Pos3 newPos = new Pos3( newX, newY, z );
		return newPos;
	}
	
	public float dot(Pos3 pos2) {
		return x * pos2.x + y * pos2.y + z * pos2.z;
	}
	
	public String toString() {
		return "x: " + x + " y: " + y + " z: " + z;
	}
}
