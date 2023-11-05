package org.team100.lib.rrt.example.full_state_arena;

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
import java.awt.geom.Ellipse2D;
import java.awt.geom.Line2D;
import java.lang.reflect.InvocationTargetException;
import java.text.NumberFormat;
import java.util.Iterator;
import java.util.List;
import java.util.Random;

import javax.swing.JComponent;
import javax.swing.JFrame;
import javax.swing.SwingUtilities;
import javax.swing.WindowConstants;

import org.team100.lib.example.Arena;
import org.team100.lib.geom.Obstacle;
import org.team100.lib.graph.LinkInterface;
import org.team100.lib.graph.Node;
import org.team100.lib.index.KDNode;
import org.team100.lib.planner.Runner;
import org.team100.lib.random.MersenneTwister;
import org.team100.lib.rrt.RRTStar7;
import org.team100.lib.space.Path;
import org.team100.lib.space.Sample;
import org.team100.lib.space.SinglePath;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;

/**
 * Four-dimensional model, plot spatial dimensions which are indices 0 and 2.
 * 
 * TODO: make both x/y and xdot/ydot pics
 */
public class FullStateArenaView extends JComponent {
    private static final boolean DEBUG = false;

    private final Runner<N4> _rrtStar;
    private final Arena<N4> _robotModel;

    private Image _backgroundImage;

    private final NumberFormat _integerFormat = NumberFormat.getIntegerInstance();

    private KDNode<Node<N4>> _T_a;
    private KDNode<Node<N4>> _T_b;

    public FullStateArenaView(Arena<N4> arena, Runner<N4> rrtStar, KDNode<Node<N4>> T_a, KDNode<Node<N4>> T_b) {
        _rrtStar = rrtStar;
        _robotModel = arena;
        _T_a = T_a;
        _T_b = T_b;
    }

    public static void main(String[] args) throws InterruptedException, InvocationTargetException {
        final JFrame frame = new JFrame();
        frame.setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);
        frame.getContentPane().setLayout(new BorderLayout());
        for (int i = 0; i < 100; ++i) {
            frame.getContentPane().removeAll();
            run(i, frame);
            Thread.sleep(250);
        }
    }

    public static void run(int ii, JFrame frame) throws InterruptedException, InvocationTargetException {
        final FullStateHolonomicArena arena = new FullStateHolonomicArena(ii);
        KDNode<Node<N4>> T_a = new KDNode<>(new Node<>(arena.initial()));
        KDNode<Node<N4>> T_b = new KDNode<>(new Node<>(arena.goal()));
        int seed = new Random().nextInt();
        System.out.printf("seed %d\n", seed);
        final RRTStar7<FullStateHolonomicArena> solver = new RRTStar7<>(arena,
                new Sample<>(arena, seed), T_a, T_b);
        solver.setRadius(6);
        // solver.SwapTrees();
        final Runner<N4> runner = new Runner<>(solver);
        final FullStateArenaView view = new FullStateArenaView(arena, runner, T_a, T_b);


        frame.getContentPane().add(view);

        SwingUtilities.invokeAndWait(new Runnable() {
            @Override
            public void run() {
                frame.setSize(1600, 800);
                frame.setVisible(true);
                frame.repaint();
            }
        });

        // RRTStar7.DEBUG = true;
        runner.runForDurationMS(200);
        //runner.runSamples(100);
        // solver.step();
        // solver.step();
        // RRTStar7.DEBUG = true;
        // System.out.println("before");
        // printTree(T_a.getValue(), 0);
        // solver.step();
        // System.out.println("after");
        // printTree(T_a.getValue(), 0);

        SinglePath<N4> bestSinglePath = runner.getBestSinglePath();
        if (bestSinglePath == null) {
            System.out.println("failed to find path");
        } else {
            System.out.println("found path");
            Random random = new MersenneTwister(new Random().nextInt());

            // so now we should try to optimize it for awhile
            // not sure which of these methods is better.

            // this just picks random points. random always finds everything there is to
            // find, additional more-focused optimization runs don't fix anything, even when
            // there are obvious corners.
            for (int i = 0; i < 1000; ++i) {
                double frac1 = random.nextDouble();
                double frac2 = random.nextDouble(frac1, 1.0);
                solver.Optimize(frac1, frac2);
            }

            System.out.printf("path distance %7.3f\n", runner.getBestSinglePath().getDistance());

        }
        System.out.println("done");

        // view.printPaths(bestPath);

        frame.repaint();
        view.repaint();

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
        if (DEBUG)
            System.out.println("doPaint");
        Arena<N4> robotModel = _robotModel;
        Matrix<N4, N1> min = robotModel.getMin();
        Matrix<N4, N1> max = robotModel.getMax();

        SinglePath<N4> bestPath = _rrtStar.getBestSinglePath();

        createBGImage(min, max, size, bestPath);

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

    /** min and max are (x xdot y ydot) */
    private void createBGImage(Matrix<N4, N1> min, Matrix<N4, N1> max, Dimension size, SinglePath<N4> link) {
        _backgroundImage = createImage(size.width, size.height);

        Graphics2D g = (Graphics2D) _backgroundImage.getGraphics();
        AffineTransform savedTransform = g.getTransform();

        double scale = setupGraphics(min, max, size, g);

        g.setColor(Color.WHITE);
        g.fillRect(0, 0, size.width, size.height);

        // obstacles
        g.setStroke(new BasicStroke(0f));
        for (Obstacle obstacle : _robotModel.obstacles()) {
            g.setColor(obstacle.color());
            g.fill(obstacle.shape());
        }

        renderRRTTree(g, scale);

        renderPaths(link, g, scale);

        g.setTransform(savedTransform);
        g.dispose();
    }

    private static final boolean renderTree = true;

    void renderEnd(Graphics2D g, KDNode<Node<N4>> tree) {
        g.setColor(Color.ORANGE);
        double x = tree.getValue().getState().get(0, 0);
        double y = tree.getValue().getState().get(2, 0);
        double r = 0.2;
        Ellipse2D.Double a = new Ellipse2D.Double(x - r, y - r, 2 * r, 2 * r);
        g.fill(a);
    }

    static void printTree(Node<N4> node, int depth) {
        Iterator<LinkInterface<N4>> i = node.getOutgoing();
        while (i.hasNext()) {
            LinkInterface<N4> out = i.next();
            if (out.get_source() != node)
                throw new IllegalArgumentException();
            Matrix<N4, N1> x_i = out.get_source().getState();
            Matrix<N4, N1> x_g = out.get_target().getState();
            for (int ii = 0; ii < depth; ++ii) {
                System.out.printf(" ");
            }
            System.out.printf("%d(%5.2f, %5.2f, %5.2f, %5.2f)->(%5.2f, %5.2f, %5.2f, %5.2f)\n", depth,
                    x_i.get(0, 0), x_i.get(1, 0), x_i.get(2, 0), x_i.get(3, 0),
                    x_g.get(0, 0), x_g.get(1, 0), x_g.get(2, 0), x_g.get(3, 0));
            printTree(out.get_target(), depth + 1);
        }
    }

    void renderTree(Graphics2D g, Node<N4> node, Color color) {
        Iterator<LinkInterface<N4>> i = node.getOutgoing();
        while (i.hasNext()) {
            LinkInterface<N4> out = i.next();
            if (out.get_source() != node)
                throw new IllegalArgumentException();
            Matrix<N4, N1> x_i = node.getState();
            Matrix<N4, N1> x_g = out.get_target().getState();
            g.setColor(color);
            g.draw(new Line2D.Double(x_i.get(0, 0), x_i.get(2, 0), x_g.get(0, 0), x_g.get(2, 0)));
            double r = 0.01;
            g.setColor(Color.BLACK);
            g.fill(new Ellipse2D.Double(x_i.get(0, 0) - r, x_i.get(2, 0) - r, 2 * r, 2 * r));
            g.fill(new Ellipse2D.Double(x_g.get(0, 0) - r, x_g.get(2, 0) - r, 2 * r, 2 * r));
            renderTree(g, out.get_target(), color);
        }
    }

    public void renderRRTTree(Graphics2D g, double scale) {
        if (!renderTree)
            return;
        if (DEBUG)
            System.out.println("renderRRTTree");
        g.setStroke(new BasicStroke((float) (1.0 / scale)));

        // paint a big circle at each end
        renderEnd(g, _T_a);
        renderEnd(g, _T_b);

        renderTree(g, _T_a.getValue(), Color.GREEN);
        renderTree(g, _T_b.getValue(), Color.RED);
    }

    void printPaths(Path<N4> path) {
        if (path == null) {
            return;
        }

        List<Matrix<N4, N1>> statesA = path.getStatesA();
        if (statesA.size() > 1) {
            Iterator<Matrix<N4, N1>> pathIter = statesA.iterator();
            Matrix<N4, N1> prev = pathIter.next();
            while (pathIter.hasNext()) {
                Matrix<N4, N1> curr = pathIter.next();
                System.out.printf("green: %f %f %f %f\n", prev.get(0, 0), prev.get(2, 0), curr.get(0, 0),
                        curr.get(2, 0));
                prev = curr;
            }
        }
        List<Matrix<N4, N1>> statesB = path.getStatesB();
        if (statesB.size() > 1) {
            Iterator<Matrix<N4, N1>> pathIter = statesB.iterator();
            Matrix<N4, N1> prev = pathIter.next();
            while (pathIter.hasNext()) {
                Matrix<N4, N1> curr = pathIter.next();
                System.out.printf("red: %f %f %f %f\n", prev.get(0, 0), prev.get(2, 0), curr.get(0, 0), curr.get(2, 0));
                prev = curr;
            }
        }

        Matrix<N4, N1> nA = statesA.get(statesA.size() - 1);
        Matrix<N4, N1> nB = statesB.get(0);
        if (nA != null && nB != null) {
            System.out.printf("black: %f %f %f %f\n", nA.get(0, 0), nA.get(2, 0), nB.get(0, 0), nB.get(2, 0));
        } else {
            System.out.println("NULLS");
        }

    }

    private void renderPaths(SinglePath<N4> path, Graphics2D g, double scale) {
        if (path == null) {
            return;
        }

        Line2D.Double line = new Line2D.Double();
        g.setStroke(new BasicStroke((float) (3.0 / scale)));

        List<SinglePath.Link<N4>> links = path.getLinks();
        if (links.size() > 1) {
            // first the line

            Iterator<SinglePath.Link<N4>> linkIter = links.iterator();
            final int linkct = links.size();
            int statei = 0;
            while (linkIter.hasNext()) {
                // TODO: make this reflect cost
                g.setColor(new Color(1, (float) statei / linkct, (float) (1.0 - (float) statei / linkct)));
                SinglePath.Link<N4> link = linkIter.next();
                line.setLine(link.x_i.get(0, 0), link.x_i.get(2, 0), link.x_g.get(0, 0), link.x_g.get(2, 0));
                g.draw(line);
                ++statei;
            }

            // then the dots

            double r = 0.02;
            g.setColor(Color.BLACK);
            linkIter = links.iterator();
            while (linkIter.hasNext()) {
                SinglePath.Link<N4> link = linkIter.next();
                g.fill(new Ellipse2D.Double(link.x_i.get(0, 0) - r, link.x_i.get(2, 0) - r, 2 * r, 2 * r));
            }

        }
    }

    /** min and max are (x xdot y ydot) */
    private double setupGraphics(Matrix<N4, N1> min, Matrix<N4, N1> max, Dimension size, Graphics2D g) {
        g.setRenderingHint(
                RenderingHints.KEY_ANTIALIASING,
                RenderingHints.VALUE_ANTIALIAS_ON);

        g.translate(min.get(0, 0), min.get(2, 0));
        double scale = Math.min(
                size.width / (max.get(0, 0) - min.get(0, 0)),
                size.height / (max.get(2, 0) - min.get(2, 0)));
        g.scale(scale, scale);
        // g.setStroke(new BasicStroke((float) (2 / scale)));
        return scale;
    }
}
