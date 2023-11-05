package org.team100.lib.rrt.example.arena;

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
import org.team100.lib.graph.Graph;
import org.team100.lib.graph.LinkInterface;
import org.team100.lib.graph.Node;
import org.team100.lib.planner.Runner;
import org.team100.lib.planner.Solver;
import org.team100.lib.rrt.RRTStar3;
import org.team100.lib.rrt.RRTStar4;
import org.team100.lib.space.Path;
import org.team100.lib.space.Sample;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

/** Two dimensional Euclidean */
public class ArenaView extends JComponent {
    private final Runner<N2> _rrtStar;
    private final Arena<N2> _robotModel;

    private Image _backgroundImage;
    private Path<N2> _bestPath = null;

    private final NumberFormat _integerFormat = NumberFormat.getIntegerInstance();

    public ArenaView(Arena<N2> arena, Runner<N2> rrtStar) {
        _rrtStar = rrtStar;
        _robotModel = arena;
    }

    public static void main(String[] args) throws InterruptedException, InvocationTargetException {
        // int ct = 10;
        int ct = 1;
        double steps = 0;
        double steps2 = 0;
        double dist = 0;
        double dist2 = 0;
        for (int i = 0; i < ct; ++i) {

            // experiment results:
            //
            // RRTStar and RRTStar2 are about the same speed;
            // the second one actually produces better paths; there's something
            // wrong with the first one.
            //
            // path distance caching is faster than recalculating the path
            // distance every time, even though it requires walking the child
            // tree on every update. updates are rare compared to queries, i guess?

            final HolonomicArena arena = new HolonomicArena(6);
            final Solver<N2> rrtstar = new RRTStar3<>(arena, new Sample<>(arena), 6);
            final Runner<N2> runner = new Runner<>(rrtstar);
            Graph.linkTypeCaching = true;

            JFrame frame = new JFrame();
            frame.setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);
            frame.getContentPane().setLayout(new BorderLayout());
            frame.getContentPane().add(new ArenaView(arena, runner));
            SwingUtilities.invokeAndWait(new Runnable() {
                @Override
                public void run() {
                    frame.setSize(1600, 800);
                    frame.setVisible(true);
                    frame.repaint();
                }
            });
            
            runner.runForDurationMS(20);
            // runner.runSamples(500);

            // final RRTStar2<HolonomicArena> rrtstar2 = new RRTStar2<>(arena, new
            // Sample(arena), 6);
            final RRTStar4<N2, HolonomicArena> rrtstar4 = new RRTStar4<>(arena, new Sample<>(arena), 6);
            final Runner<N2> runner2 = new Runner<>(rrtstar4);
            Graph.linkTypeCaching = true;
            JFrame frame2 = new JFrame();
            frame2.setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);
            frame2.getContentPane().setLayout(new BorderLayout());
            frame2.getContentPane().add(new ArenaView(arena, runner2));
            SwingUtilities.invokeAndWait(new Runnable() {
                @Override
                public void run() {
                    frame2.setSize(1600, 800);
                    frame2.setVisible(true);
                    frame2.repaint();
                }
            });
            
            runner2.runForDurationMS(20);
            // runner.runSamples(500);

            System.out.printf("2 steps %d distance %5.2f\n",
                    runner2.getStepNo(), runner2.getBestPath().getDistance());
            System.out.printf("full distance %5.2f\n", rrtstar4.getFullBestPath().getDistance());
            System.out.printf("1 steps %d distance %5.2f --- 2 steps %d distance %5.2f\n",
                    runner.getStepNo(), runner.getBestPath().getDistance(),
                    runner2.getStepNo(), runner2.getBestPath().getDistance());
            steps += runner.getStepNo();
            steps2 += runner2.getStepNo();
            dist += runner.getBestPath().getDistance();
            dist2 += runner2.getBestPath().getDistance();
        }
        System.out.println("means");
        System.out.printf("1 steps %7.2f distance %5.2f --- 2 steps %7.2f distance %5.2f\n",
                steps / ct, dist / ct, steps2 / ct, dist2 / ct);
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
        Matrix<N2, N1> min = robotModel.getMin();
        Matrix<N2, N1> max = robotModel.getMax();

        Path<N2> bestPath = _rrtStar.getBestPath();

        if (_backgroundImage == null ||
                _backgroundImage.getWidth(null) != size.width ||
                _backgroundImage.getHeight(null) != size.height ||
                Path.isBetter(bestPath, _bestPath)) {
            createBGImage(min, max, size, bestPath);
            _bestPath = bestPath;
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

    private void createBGImage(Matrix<N2, N1> min, Matrix<N2, N1> max, Dimension size, Path<N2> link) {
        _backgroundImage = createImage(size.width, size.height);

        Graphics2D g = (Graphics2D) _backgroundImage.getGraphics();
        AffineTransform savedTransform = g.getTransform();

        double scale = setupGraphics(min, max, size, g);

        g.setColor(Color.WHITE);
        g.fillRect(0, 0, size.width, size.height);

        // obstacles
        g.setStroke(new BasicStroke(0f));
        // g.setColor(new Color(0x8888ff));
        for (Obstacle obstacle : _robotModel.obstacles()) {
            g.setColor(obstacle.color());
            g.fill(obstacle.shape());
        }

        renderRRTTree(g);

        renderPaths(link, g, scale);

        g.setTransform(savedTransform);
        g.dispose();
    }

    private void renderRRTTree(Graphics2D g) {
        Line2D.Double line = new Line2D.Double();

        for (Node<N2> node : _rrtStar.getNodesA()) {
            LinkInterface<N2> incoming = node.getIncoming();
            if (incoming != null) {
                Node<N2> parent = incoming.get_source();
                Matrix<N2, N1> n = node.getState();
                Matrix<N2, N1> p = parent.getState();
                g.setColor(Color.GREEN);
                line.setLine(n.get(0, 0), n.get(1, 0), p.get(0, 0), p.get(1, 0));
                g.draw(line);
            }
        }
        for (Node<N2> node : _rrtStar.getNodesB()) {
            LinkInterface<N2> incoming = node.getIncoming();
            if (incoming != null) {
                Node<N2> parent = incoming.get_source();
                Matrix<N2, N1> n = node.getState();
                Matrix<N2, N1> p = parent.getState();
                g.setColor(Color.RED);
                line.setLine(n.get(0, 0), n.get(1, 0), p.get(0, 0), p.get(1, 0));
                g.draw(line);
            }
        }
    }

    private void renderPaths(Path<N2> path, Graphics2D g, double scale) {
        if (path == null) {
            return;
        }

        Line2D.Double line = new Line2D.Double();
        g.setStroke(new BasicStroke((float) (5 / scale)));

        if (path.getStatesA().size() > 1) {
            Iterator<Matrix<N2, N1>> pathIter = path.getStatesA().iterator();
            Matrix<N2, N1> prev = pathIter.next();
            while (pathIter.hasNext()) {
                Matrix<N2, N1> curr = pathIter.next();
                g.setColor(Color.GREEN);
                line.setLine(prev.get(0, 0), prev.get(1, 0), curr.get(0, 0), curr.get(1, 0));
                g.draw(line);
                prev = curr;
            }
        }
        if (path.getStatesB().size() > 1) {
            Iterator<Matrix<N2, N1>> pathIter = path.getStatesB().iterator();
            Matrix<N2, N1> prev = pathIter.next();
            while (pathIter.hasNext()) {
                Matrix<N2, N1> curr = pathIter.next();
                g.setColor(Color.RED);
                line.setLine(prev.get(0, 0), prev.get(1, 0), curr.get(0, 0), curr.get(1, 0));
                g.draw(line);
                prev = curr;
            }
        }
    }

    private double setupGraphics(Matrix<N2, N1> min, Matrix<N2, N1> max, Dimension size, Graphics2D g) {
        g.setRenderingHint(
                RenderingHints.KEY_ANTIALIASING,
                RenderingHints.VALUE_ANTIALIAS_ON);

        g.translate(min.get(0, 0), min.get(1, 0));
        double scale = Math.min(
                size.width / (max.get(0, 0) - min.get(0, 0)),
                size.height / (max.get(1, 0) - min.get(1, 0)));
        g.scale(scale, scale);
        g.setStroke(new BasicStroke((float) (0.25 / scale)));
        return scale;
    }

    static Color brighter(Color color) {
        float[] hsb = Color.RGBtoHSB(color.getRed(), color.getGreen(), color.getBlue(), null);
        hsb[1] = Math.max(0.0f, hsb[1] - 0.25f);
        hsb[2] = Math.min(1.0f, hsb[2] + 0.25f);
        color = Color.getHSBColor(hsb[0], hsb[1], hsb[2]);
        return color;
    }
}
