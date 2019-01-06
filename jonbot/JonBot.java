package jonbot;
import robocode.*;
import robocode.util.Utils;

import java.awt.Color;
import java.awt.geom.Point2D;

// API help : https://robocode.sourceforge.io/docs/robocode/robocode/Robot.html

/**
 * OneHourChallengeBot - a robot by (your name here)
 */
public class JonBot extends AdvancedRobot
{
    private static final double WALL_BUFFER = 60;
    private static final double WALL_ADJUST_RADS = Math.PI/4;

    // Compas directions (up, down, left, right) in radians
    private static final double UP = 0;
    private static final double UP_FULL = 2 * Math.PI;
    private static final double RIGHT = Math.PI/2;
    private static final double DOWN = Math.PI;
    private static final double LEFT = 1.5 * Math.PI;


    private double firePower;

    /**
     * run: JonBot's default behavior
     */
    public void run() {
        this.firePower = 1;
        this.setAdjustGunForRobotTurn(true);
        this.setAdjustRadarForGunTurn(true);
	    // Initialization of the robot should be put here

	    // After trying out your robot, try uncommenting the import at the top,
	    // and the next line:

	    this.setColors(Color.red,Color.blue,Color.green); // body,gun,radar

	    // Robot main loop
	    while(true) {
            this.turnRadarRight(Double.POSITIVE_INFINITY);
	    }
    }

    private double getAdjustedBearingToEnemy(ScannedRobotEvent e, double objectHeading) {
        double enemyBearingToMyLocation = e.getBearingRadians() + this.getHeadingRadians();
        double radarDelta = enemyBearingToMyLocation - objectHeading;
        return Utils.normalRelativeAngle(radarDelta) * 2;
    }


    private double adjustHeadingAvoidCollisions(double proposedHeading) {
        // too close to top wall
        double heading = this.getHeadingRadians();
        if(this.getBattleFieldHeight() - this.getY() < WALL_BUFFER) {
            if(heading > UP && heading < RIGHT) {
                // if facing diagonally to the up/right
                return proposedHeading + WALL_ADJUST_RADS;
            } else if(heading > LEFT && heading < UP_FULL) {
                // if facing diagonally to the up/left
                return proposedHeading - WALL_ADJUST_RADS;
            }
        }

        // too close to left wall
        if(this.getX() < WALL_BUFFER) {
            if(heading > LEFT && heading < UP_FULL) {
                // if facing diagonally left/up
                return proposedHeading + WALL_ADJUST_RADS;
            } else if(heading > DOWN && heading < LEFT) {
                return proposedHeading - WALL_ADJUST_RADS;
            }
        }

        // too close to bottom wall
        if(this.getY() - this.getBattleFieldHeight() < WALL_ADJUST_RADS) {
            if(heading < DOWN && heading > RIGHT) {
                // if facing diagonally left/up
                return proposedHeading - WALL_ADJUST_RADS;
            } else if(heading > DOWN && heading < LEFT) {
                return proposedHeading + WALL_ADJUST_RADS;
            }
        }

        // too close to right wall
        if(this.getX() - this.getBattleFieldWidth() < WALL_ADJUST_RADS) {
            if(heading > UP && heading < RIGHT) {
                // if facing diagonally left/up
                return proposedHeading - WALL_ADJUST_RADS;
            } else if(heading > RIGHT && heading < DOWN) {
                return proposedHeading + WALL_ADJUST_RADS;
            }
        }

        return proposedHeading;
    }

    public void onHitRobot(HitRobotEvent e) {
    }


    /**
     * onScannedRobot: What to do when you see another robot
     */
    public void onScannedRobot(ScannedRobotEvent e) {
	    // Replace the next line with any behavior you would like
        this.setTurnRadarRightRadians(this.getAdjustedBearingToEnemy(e, this.getRadarHeadingRadians()));
        this.setTurnGunRightRadians(this.getAdjustedBearingToEnemy(e, this.getGunHeadingRadians()));

        double updatedPlayerHeading = this.getAdjustedBearingToEnemy(e, this.getHeadingRadians());
        double collisionAvoidedHeading = this.adjustHeadingAvoidCollisions(updatedPlayerHeading);
        this.setTurnRightRadians(collisionAvoidedHeading);

        double power = this.calculateFirePower(e.getDistance());
        out.println("Firing with power: " + power);
        fire(power);

        setAhead(30);
    }


    private double calculateFirePower(double distanceToEnemy) {
        double maxDistance = Math.sqrt(Math.pow(this.getBattleFieldHeight(), 2) + Math.pow(this.getBattleFieldWidth(), 2));
        return Rules.MAX_BULLET_POWER - clamp(0, maxDistance, 0.1, 3.0, distanceToEnemy);
    }

    private static double clamp(double inputMin, double inputMax, double outputMin, double outputMax, double value) {
        return (((outputMax - outputMin) * (value - inputMin)) / (inputMax - inputMin)) + outputMin;
    }


    /**
     * onHitByBullet: What to do when you're hit by a bullet
     */
    public void onHitByBullet(HitByBulletEvent e) {
	    // Replace the next line with any behavior you would like
    }
	
    /**
     * onHitWall: What to do when you hit a wall
     */
    public void onHitWall(HitWallEvent e) {
	    // Replace the next line with any behavior you would like
    }
}
