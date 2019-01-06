package jons;
import robocode.*;
import java.awt.Color;

// API help : https://robocode.sourceforge.io/docs/robocode/robocode/Robot.html

/**
 * OneHourChallengeBot - a robot by (your name here)
 */
public class OneHourChallengeBot extends Robot
{
    /**
     * run: OneHourChallengeBot's default behavior
     */
    public void run() {
	// Initialization of the robot should be put here

	// After trying out your robot, try uncommenting the import at the top,
	// and the next line:

	setColors(Color.red,Color.blue,Color.green); // body,gun,radar

	// Robot main loop
	while(true) {
	    // Replace the next 4 lines with any behavior you would like
	    turnRadarRight(5);
	    ahead(20);
	    // turnGunRight(360);
	    // back(100);
	    // turnGunRight(360);
	}
    }

    /**
     * onScannedRobot: What to do when you see another robot
     */
    public void onScannedRobot(ScannedRobotEvent e) {
	// Replace the next line with any behavior you would like
	double b = e.getBearing();
	if(b > 0) {
	    turnRight(b);
	} else if(b < 0) {
	    turnLeft(-b);
	}

		
	fire(1);
    }

    /**
     * onHitByBullet: What to do when you're hit by a bullet
     */
    public void onHitByBullet(HitByBulletEvent e) {
	// Replace the next line with any behavior you would like

	back(10);
    }
	
    /**
     * onHitWall: What to do when you hit a wall
     */
    public void onHitWall(HitWallEvent e) {
	// Replace the next line with any behavior you would like
	back(20);
    }	
}
