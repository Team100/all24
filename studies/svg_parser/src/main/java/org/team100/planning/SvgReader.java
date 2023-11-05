package org.team100.planning;

import java.io.IOException;
import java.io.InputStream;

import org.team100.plotter.SVGToPlotOperations;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;
import org.w3c.dom.svg.SVGDocument;
import org.w3c.dom.svg.SVGPathSeg;
import org.w3c.dom.svg.SVGPathSegList;

import io.sf.carte.echosvg.anim.dom.SAXSVGDocumentFactory;
import io.sf.carte.echosvg.anim.dom.SVGDOMImplementation;
import io.sf.carte.echosvg.anim.dom.SVGOMPathElement;
import io.sf.carte.echosvg.dom.svg.AbstractSVGPathSegList;
import io.sf.carte.echosvg.dom.svg.SVGPathSegItem;

/** Scan the input stream and give SVG path operators to the transformer. */
public class SvgReader {
    private final InputStream input;
    private final SVGToPlotOperations transformer;

    /**
     * @param input SVG
     * @param transformer accepts path operators
     */
    public SvgReader(InputStream input, SVGToPlotOperations transformer) {
        this.input = input;
        this.transformer = transformer;
    }

    public void run() {
        NodeList nodeList = getPaths();
        int nodeListLength = nodeList.getLength();
        for (int i = 0; i < nodeListLength; ++i) {
            System.out.println("path " + i);
            SVGOMPathElement node = (SVGOMPathElement) nodeList.item(i);
            SVGPathSegList pathList = node.getNormalizedPathSegList();
            int pathListLength = pathList.getNumberOfItems();
            for (int j = 0; j < pathListLength; ++j) {
                SVGPathSeg item = pathList.getItem(j);
                handleItem(item);
            }
        }
        transformer.end();
    }

    private NodeList getPaths() {
        try {
            String svgNS = SVGDOMImplementation.SVG_NAMESPACE_URI;
            SAXSVGDocumentFactory factory = new SAXSVGDocumentFactory(null);
            SVGDocument doc = factory.createDocument(svgNS, input);
            Element documentElement = doc.getDocumentElement();
            return documentElement.getElementsByTagName("path");
        } catch (IOException e) {
            e.printStackTrace();
            return new NodeList() {
                @Override
                public int getLength() {
                    return 0;
                }

                @Override
                public Node item(int arg0) {
                    return null;
                }
            };
        }
    }

    private void handleItem(SVGPathSeg item) {
        if (item instanceof AbstractSVGPathSegList.SVGPathSegMovetoLinetoItem) {
            AbstractSVGPathSegList.SVGPathSegMovetoLinetoItem m = (AbstractSVGPathSegList.SVGPathSegMovetoLinetoItem) item;
            switch (m.getPathSegType()) {
                case SVGPathSeg.PATHSEG_LINETO_ABS:
                    handleL(m);
                    break;
                case SVGPathSeg.PATHSEG_MOVETO_ABS:
                    handleM(m);
                    break;
                default:
                    System.out.printf("Unknown Move/Line: %s %5.3f %5.3f\n", m.getPathSegTypeAsLetter(), m.getX(),
                            m.getY());
            }
        } else if (item instanceof AbstractSVGPathSegList.SVGPathSegCurvetoCubicItem) {
            AbstractSVGPathSegList.SVGPathSegCurvetoCubicItem m = (AbstractSVGPathSegList.SVGPathSegCurvetoCubicItem) item;
            switch (m.getPathSegType()) {
                case SVGPathSeg.PATHSEG_CURVETO_CUBIC_ABS:
                    handleC(m);
                    break;
                default:
                    System.out.printf("Unknown Curve: %s %5.3f %5.3f %5.3f %5.3f %5.3f %5.3f\n",
                            m.getPathSegTypeAsLetter(),
                            m.getX(), m.getY(),
                            m.getX1(), m.getY1(),
                            m.getX2(), m.getY2());

            }
        } else if (item instanceof SVGPathSegItem) {
            SVGPathSegItem m = (SVGPathSegItem) item;
            switch (m.getPathSegType()) {
                case SVGPathSeg.PATHSEG_CLOSEPATH:
                    handleZ();
                    break;
                default:
                    System.out.println("Unknown item letter: " + item.getPathSegTypeAsLetter());
            }
        } else {
            System.out.println("unknown item type: " + item.getClass().getName());
        }
    }

    private void handleZ() {
        System.out.println("close path");
        transformer.close();
    }

    private void handleC(AbstractSVGPathSegList.SVGPathSegCurvetoCubicItem m) {
        System.out.printf("Curve Abs: %5.3f %5.3f %5.3f %5.3f %5.3f %5.3f\n",
                m.getX(), m.getY(), m.getX1(), m.getY1(), m.getX2(), m.getY2());
        transformer.curve(
                m.getX(), m.getY(),
                m.getX1(), m.getY1(),
                m.getX2(), m.getY2());
    }

    private void handleM(AbstractSVGPathSegList.SVGPathSegMovetoLinetoItem m) {
        System.out.printf("Move Abs: %5.3f %5.3f\n", m.getX(), m.getY());
        transformer.move(m.getX(), m.getY());
    }

    private void handleL(AbstractSVGPathSegList.SVGPathSegMovetoLinetoItem m) {
        System.out.printf("Line Abs: %5.3f %5.3f\n", m.getX(), m.getY());
        transformer.line(m.getX(), m.getY());
    }

}