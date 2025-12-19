package stark.planning;

import stark.DefaultRandomGenerator;

import javax.swing.*;
import java.awt.*;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Line2D;
import java.awt.geom.Rectangle2D;
import java.util.List;

public class RRTstar_vis extends JPanel {

    private static final int MAP_SIZE = 300;

    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);

        // Centering offset to draw state space in the middle
        int offsetX = (getWidth() - MAP_SIZE) / 2;
        int offsetY = (getHeight() - MAP_SIZE) / 2;

        // Draw the state space boundary
        Graphics2D g2 = (Graphics2D) g;
        g2.setColor(Color.GRAY);
        g2.drawRect(offsetX, offsetY, MAP_SIZE, MAP_SIZE);

        // Draw obstacles
        if (RRTstar.obstacles != null) {
            g2.setColor(Color.BLACK);
            //Graphics2D g2 = (Graphics2D) g;
            for (Obstacle p : RRTstar.obstacles) {
                Rectangle2D rect = new Rectangle2D.Double(p.getX() + offsetX, p.getY() + offsetY, p.getW(), p.getW());
                g2.fill(rect);
            }
        }

        // Draw RRT tree edges
        g2.setColor(Color.LIGHT_GRAY);

        for (Pos[] edge : RRTstar.treeEdges) {
            g2.draw(new Line2D.Double(edge[0].getX() + offsetX, edge[0].getY() + offsetY,
                    edge[1].getX() + offsetX, edge[1].getY() + offsetY));
        }

        // Draw final path in blue
        g2.setColor(Color.BLUE);
        List<Pos> path = RRTstar.finalPath;
        for (int i = 0; i < path.size() - 1; i++) {
            Pos p1 = path.get(i);
            Pos p2 = path.get(i + 1);
            g2.draw(new Line2D.Double(p1.getX() + offsetX, p1.getY() + offsetY,
                    p2.getX() + offsetX, p2.getY() + offsetY));
        }

        // ✅ Only draw start and goal if they are set
        if (RRTstar.startP != null && RRTstar.endP != null) {
            g2.setColor(Color.RED);
            Shape oval = new Ellipse2D.Double(RRTstar.startP.getX() + offsetX - 5, RRTstar.startP.getY() + offsetY - 5, 10, 10);
            g2.fill(oval);
            g2.setColor(Color.YELLOW);
            Shape oval2 = new Ellipse2D.Double(RRTstar.endP.getX() + offsetX - 5, RRTstar.endP.getY() + offsetY - 5, 10, 10);
            g2.fill(oval2);
        }
    }

    public static void main(String[] args) {
        JFrame frame = new JFrame("RRT Pathfinding");
        RRTstar_vis panel = new RRTstar_vis();
        frame.add(panel);
        frame.setSize(MAP_SIZE + 10, MAP_SIZE + 10);
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setVisible(true);
        DefaultRandomGenerator rand = new DefaultRandomGenerator();

        List<Obstacle> obstacleList = RRTstar.generateObstacles(rand);
        double dimension = 3;
        Pos start = RRTstar.generateFreePos(rand, obstacleList,dimension);
        Goal goal = new Goal(RRTstar.generateFreePos(rand, obstacleList,dimension));
        RRTstar.runEnvironment(rand, panel, start, goal, obstacleList,dimension);
    }

}

