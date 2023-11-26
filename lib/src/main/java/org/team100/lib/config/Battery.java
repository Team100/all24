package org.team100.lib.config;

/**
 * Represents the identity of the battery as perceived by the barcode reader.
 * 
 * This is an enum in order to keep metadata here, as fields.
 * 
 * Downstream analysis uses the id numbers; they should not be reused.
 */
public enum Battery {
    ONE(1, Use.COMP),
    TWO(2, Use.COMP),
    THREE(3, Use.PRACTICE),
    UNKNOWN(0, Use.PROTOTYPE);

    /** To enforce usage rules. */
    public enum Use {
        COMP,
        PRACTICE,
        PROTOTYPE
    }

    public final int id;
    public final Use use;

    private Battery(int id, Use use) {
        this.id = id;
        this.use = use;
    }
}
