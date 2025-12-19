package stark.planning;

import org.apache.commons.math3.random.RandomGenerator;

import javax.swing.*;
import java.awt.geom.Rectangle2D;
import java.util.*;
import java.util.List;

public class RRTstar {
    public static final int MAP_SIZE = 300;
    public static final int OBSTACLES_SIZE = 70;
    public static final int OBSTACLES_HEIGHT = 7;
    public static final int NUM_OBSTACLES = 10;
    public static final int MOVEMENT = 5;

    public static final List<Pos> treeNodes = new ArrayList<>();
    public static final List<Pos[]> treeEdges = new ArrayList<>();
    public static final List<Pos> finalPath = new ArrayList<>();

    public static Pos startP;
    public static Goal endP;

    public static List<Obstacle> obstacles;


    // Create 3 different environments with obstacles
    @SuppressWarnings("unchecked")
    public static List<Obstacle> generateObstacles(RandomGenerator rand) {
        obstacles = new ArrayList<>();
        while (obstacles.size() < NUM_OBSTACLES) {
            Obstacle obs = new Obstacle(new Pos(rand.nextDouble()*(MAP_SIZE-OBSTACLES_SIZE), rand.nextDouble()*(MAP_SIZE-OBSTACLES_SIZE)), rand.nextDouble()*OBSTACLES_HEIGHT, rand.nextDouble()*OBSTACLES_SIZE);
            if (isValidObstacle(obs.getPos(), obstacles)) {
                    obstacles.add(obs);
            }
        }
        return obstacles;
    }

    public static void runEnvironment(RandomGenerator rand, JPanel panel, Pos start, Goal goal, List<Obstacle> obstacleList, double dimension) {
        treeNodes.clear();
        treeEdges.clear();
        finalPath.clear();
        startP = start;
        endP = goal;
        rrtSearch(rand, startP, endP, obstacleList, dimension);
        panel.repaint();
    }

    // Ensure obstacle does not overlap with others
    public static boolean isValidObstacle(Pos p, List<Obstacle> obstacleList) {
        boolean in = true;
        for (int i = 0; i < obstacleList.size(); i++) {
            //Rectangle2D r = new Rectangle2D.Double(o.getX(), o.getY(), o.getW(), o.getW());
            if (p.getX() >= obstacleList.get(i).getX()-obstacleList.get(i).getW() && p.getX() <= obstacleList.get(i).getX() + obstacleList.get(i).getW() &&
                    p.getY() >= obstacleList.get(i).getY()-obstacleList.get(i).getW() && p.getY() <= obstacleList.get(i).getY() + obstacleList.get(i).getW()){
                in = false;
            }
        }
        return in;
    }

    // Create a random free point not inside any obstacle
    public static Pos generateFreePos(RandomGenerator rand, List<Obstacle> obstacleList, double dimension) {
        Pos p;
        do {
            p = new Pos(rand.nextDouble()*MAP_SIZE, rand.nextDouble()*MAP_SIZE);
        } while (!isFree(p,obstacleList, dimension));
        return p;
    }

    // Check if a point is free (not inside obstacles)
    public static boolean isFree(Pos p, List<Obstacle> obstacleList, double dimension) {
        boolean free = true;
        //for (Obstacle o : obstacles) {
        //    Rectangle2D r = new Rectangle2D.Double(o.getX(), o.getY(), o.getW(), o.getW());
        //    if (r.contains(p.getX(), p.getY())) {
        //        free = false;
        //    }
        //}
        for (int i = 0; i < obstacleList.size(); i++) {
            if (p.getX() >= obstacleList.get(i).getX()-obstacleList.get(i).getW() - dimension && p.getX() <= obstacleList.get(i).getX() + obstacleList.get(i).getW() + dimension &&
                    p.getY() >= obstacleList.get(i).getY()-obstacleList.get(i).getW() - dimension && p.getY() <= obstacleList.get(i).getY() + obstacleList.get(i).getW() + dimension){
                free = false;
            }
        }
        return free;
    }

    // Main RRT algorithm: expands tree toward random samples until goal is reached
    public static void rrtSearch(RandomGenerator rand, Pos start, Goal goal, List<Obstacle> obstacleList, double dimension) {
        Map<Pos, Pos> parentMap = new HashMap<>();
        treeNodes.add(start);

        boolean goalConnected = false;

        while (!goalConnected) {
            Pos randPoint;
            Pos lastPoint = treeNodes.get(treeNodes.size()-1);
            double weight = distance(lastPoint,goal.getPos());
            do {
                randPoint = new Pos(rand.nextDouble()*MAP_SIZE, rand.nextDouble()*MAP_SIZE);
            } while (!isFree(randPoint,obstacleList,dimension) || distance(randPoint,goal.getPos()) >= weight);

            Pos nearest = getNearest(randPoint);
            Pos newPoint = simulateMovement(nearest, randPoint, obstacleList, dimension);

            if (!newPoint.equals(nearest) && isCollisionFree(nearest, newPoint, obstacleList, dimension)) {
                treeNodes.add(newPoint);
                treeEdges.add(new Pos[]{nearest, newPoint});
                parentMap.put(newPoint, nearest);
            }
            Pos closestToGoal = getNearest(goal.getPos());
            if (distance(closestToGoal, goal.getPos()) < MOVEMENT && isCollisionFree(closestToGoal, goal.getPos(), obstacleList, dimension)) {
                parentMap.put(goal.getPos(), closestToGoal);
                treeEdges.add(new Pos[]{closestToGoal, goal.getPos()});
                goalConnected = true;
                Pos current = goal.getPos();
                while (current != null) {
                    finalPath.add(0, current);
                    current = parentMap.get(current);
                }
                System.out.println("Path found!");
            }
        }
    }

    public static Pos getNearest(Pos target) {
        Pos best = null;
        double minDist = Double.MAX_VALUE;
        for (Pos p : treeNodes) {
            double dist = distance(p, target);
            if (dist < minDist) {
                minDist = dist;
                best = p;
            }
        }
        return best;
    }


    public static Pos simulateMovement(Pos from, Pos to, List<Obstacle> obstacleList, double dimension) {
        double angle = Math.atan2(to.getY() - from.getY(), to.getX() - from.getX());
        double newX = from.getX() + MOVEMENT * Math.cos(angle);
        double newY = from.getY() + MOVEMENT * Math.sin(angle);
        Pos p = new Pos(newX, newY);
        return isFree(p,obstacleList,dimension) ? p : from;
    }


    public static boolean isCollisionFree(Pos a, Pos b, List<Obstacle> obstacleList, double dimension) {
        double steps = 100;
        boolean free = true;
        for (int i = 0; i <= steps; i++) {
            double t = i / steps;
            double x = a.getX() * (1 - t) + b.getX() * t;
            double y = a.getY() * (1 - t) + b.getY() * t;
            if (!isFree(new Pos(x, y), obstacleList, dimension)) {
                free = false;
            }
        }
        return free;
    }

    // Calculate Euclidean distance
    public static double distance(Pos a, Pos b) {
        return Math.hypot(a.getX() - b.getX(), a.getY() - b.getY());
    }
}
