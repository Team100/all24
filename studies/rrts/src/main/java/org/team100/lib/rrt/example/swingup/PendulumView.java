package org.team100.lib.rrt.example.swingup;
import java.awt.BasicStroke;
import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.FontMetrics;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Image;
import java.awt.RenderingHints;
import java.awt.geom.AffineTransform;
import java.awt.geom.Line2D;
import java.lang.reflect.InvocationTargetException;
import java.text.NumberFormat;
import java.util.Iterator;

import javax.swing.JComponent;
import javax.swing.JFrame;
import javax.swing.SwingUtilities;
import javax.swing.WindowConstants;

import org.team100.lib.example.Arena;
import org.team100.lib.geom.Obstacle;
import org.team100.lib.graph.LinkInterface;
import org.team100.lib.graph.Node;
import org.team100.lib.planner.Runner;
import org.team100.lib.rrt.RRTStar5;
import org.team100.lib.space.Path;
import org.team100.lib.space.Sample;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

public class PendulumView extends JComponent {
    private final Runner<N2> _rrtStar;
    private final Arena<N2> _robotModel;
    private int framecounter;

    private Image _backgroundImage;

    private final NumberFormat _integerFormat = NumberFormat.getIntegerInstance();

    public PendulumView(Arena<N2> arena, Runner<N2> rrtStar) {
        _rrtStar = rrtStar;
        _robotModel = arena;
    }


    public static void main(String[] args) throws InterruptedException, InvocationTargetException {
        Matrix<N2,N1> init = new Matrix<>(Nat.N2(),Nat.N1(), new double[] { 0, 0 });
        Matrix<N2,N1> goal = new Matrix<>(Nat.N2(),Nat.N1(), new double[] { Math.PI, 0 });
        double gravity = 9.81;
        final Arena<N2> arena = new PendulumArena2(init, goal, gravity);
        final RRTStar5<Arena<N2>> worker = new RRTStar5<>(arena, new Sample<>(arena), 1);
        final Runner<N2> rrtStar = new Runner<>(worker);
        JFrame frame = new JFrame();
        frame.setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);
        frame.getContentPane().setLayout(new BorderLayout());
        frame.getContentPane().add(new PendulumView(arena, rrtStar));

        SwingUtilities.invokeAndWait(new Runnable() {
            @Override
            public void run() {
                frame.setSize(600, 600);
                frame.setVisible(true);
                frame.repaint();
            }
        });
        // it should work both ways, time-reversed in this case:
        //worker.SwapTrees();
        rrtStar.runForDurationMS(1000);
        //rrtStar.runSamples(3000);
        Path<N2> bestPath = rrtStar.getBestPath();
        if (bestPath != null) {
            System.out.println(bestPath);
        }
        System.out.println("done");
    }




    @Override
    protected void paintComponent(Graphics graphics) {
        doPaint((Graphics2D) graphics, getSize());

        SwingUtilities.invokeLater(new Runnable() {
            @Override
            public void run() {
                repaint();
            }
        });
    }

    public void doPaint(Graphics2D g, Dimension size) {

        Arena<N2> robotModel = _robotModel;
        Matrix<N2,N1> min = robotModel.getMin();
        Matrix<N2,N1> max = robotModel.getMax();

        Path<N2> bestPath = _rrtStar.getBestPath();

        framecounter += 1;
        if (framecounter > 100) {
            framecounter = 0;
            createBGImage(min, max, size, bestPath);
        }

        g.drawImage(_backgroundImage, 0, 0, null);

        AffineTransform savedTransform = g.getTransform();

        g.setTransform(savedTransform);

        g.setColor(Color.WHITE);
        FontMetrics fm = g.getFontMetrics();
        String count = _integerFormat.format(_rrtStar.getStepNo());
        g.drawString(count, 4, 4 + fm.getAscent());
        g.setColor(Color.BLACK);
        g.drawString(count, 3, 3 + fm.getAscent());
    }

    private void createBGImage(Matrix<N2,N1> min, Matrix<N2,N1> max, Dimension size, Path<N2> link) {
        _backgroundImage = createImage(size.width, size.height);

        Graphics2D g = (Graphics2D) _backgroundImage.getGraphics();
        AffineTransform savedTransform = g.getTransform();

        AffineTransform transform = AffineTransform.getTranslateInstance(size.width / 2, size.height / 2);
        transform.scale(1, -1);
        g.setTransform(transform);

        setupGraphics(min, max, size, g);

        g.setColor(Color.WHITE);
        g.fillRect(-size.width / 2, -size.height / 2, size.width, size.height);

        // obstacles
        g.setStroke(new BasicStroke(0f));
        for (Obstacle obstacle : _robotModel.obstacles()) {
            g.setColor(obstacle.color());
            g.fill(obstacle.shape());
        }

        renderRRTTree(g);

        renderPaths(link, g);

        g.setTransform(savedTransform);
        g.dispose();
    }

    private void renderRRTTree(Graphics2D g) {
        Line2D.Double line = new Line2D.Double();
        g.setStroke(new BasicStroke((float) 0.01));
        for (Node<N2> node : _rrtStar.getNodesA()) {
            LinkInterface<N2> incoming = node.getIncoming();
            if (incoming != null) {
                Node<N2> parent = incoming.get_source();
                Matrix<N2,N1> n = node.getState();
                Matrix<N2,N1> p = parent.getState();
                g.setColor(Color.GREEN);
                line.setLine(n.get(0,0), n.get(1,0), p.get(0,0), p.get(1,0));
                g.draw(line);
            }
        }
        for (Node<N2> node : _rrtStar.getNodesB()) {
            LinkInterface<N2> incoming = node.getIncoming();
            if (incoming != null) {
                Node<N2> parent = incoming.get_source();
                Matrix<N2,N1> n = node.getState();
                Matrix<N2,N1> p = parent.getState();
                g.setColor(Color.RED);
                line.setLine(n.get(0,0), n.get(1,0), p.get(0,0), p.get(1,0));
                g.draw(line);
            }
        }
    }

    private void renderPaths(Path<N2> path, Graphics2D g) {
        if (path == null) {
            return;
        }

        Line2D.Double line = new Line2D.Double();
        g.setStroke(new BasicStroke((float) 0.1));

        if (path.getStatesA().size() > 1) {
            Iterator<Matrix<N2,N1>> pathIter = path.getStatesA().iterator();
            Matrix<N2,N1> prev = pathIter.next();
            while (pathIter.hasNext()) {
                Matrix<N2,N1> curr = pathIter.next();
                g.setColor(brighter(Color.GREEN));
                line.setLine(prev.get(0,0), prev.get(1,0), curr.get(0,0), curr.get(1,0));
                g.draw(line);
                prev = curr;
            }
        }
        if (path.getStatesB().size() > 1) {
            Iterator<Matrix<N2,N1>> pathIter = path.getStatesB().iterator();
            Matrix<N2,N1> prev = pathIter.next();
            while (pathIter.hasNext()) {
                Matrix<N2,N1> curr = pathIter.next();
                g.setColor(brighter(Color.RED));
                line.setLine(prev.get(0,0), prev.get(1,0), curr.get(0,0), curr.get(1,0));
                g.draw(line);
                prev = curr;
            }
        }
    }

    private void setupGraphics(Matrix<N2,N1> min, Matrix<N2,N1> max, Dimension size, Graphics2D g) {
        g.setRenderingHint(
                RenderingHints.KEY_ANTIALIASING,
                RenderingHints.VALUE_ANTIALIAS_ON);

        g.translate(min.get(0,0), min.get(1,0));
        double xscale = size.width / (max.get(0,0) - min.get(0,0));
        double yscale = size.height / (max.get(1,0) - min.get(1,0));
        g.scale(xscale, yscale);
        g.setStroke(new BasicStroke((float) 0.01));
    }

    static Color brighter(Color color) {
        float[] hsb = Color.RGBtoHSB(color.getRed(), color.getGreen(), color.getBlue(), null);
        hsb[1] = Math.max(0.0f, hsb[1] - 0.25f);
        hsb[2] = Math.min(1.0f, hsb[2] + 0.25f);
        color = Color.getHSBColor(hsb[0], hsb[1], hsb[2]);
        return color;
    }

}
