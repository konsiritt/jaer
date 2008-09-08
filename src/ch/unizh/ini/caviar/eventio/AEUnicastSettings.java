/*
 * AEUnicastDialog.java
 *
 * Created on April 25, 2008, 8:40 AM
 */
/** Interface for AEUnicast connections, to allow use of common dialog setting
@author tobi
 */
package ch.unizh.ini.caviar.eventio;

public interface AEUnicastSettings {

    /** Default address first (versus timestamp first) setting */
    public static final boolean DEFAULT_ADDRESS_FIRST = true;
    /** Default is to use sequence numbers as first byte of each packet */
    public static final boolean DEFAULT_USE_SEQUENCE_NUMBER = true;
    
    
    public static final String DEFAULT_HOST = "localhost";
    /** Default jAER UDP port */
    public static final int DEFAULT_PORT = AENetworkInterface.DATAGRAM_PORT;
    /** Default swapping of bytes */
    public static final boolean DEFAULT_SWAPBYTES_ENABLED = false;
    
    /** Default timestamp multiplier */
    public static final float DEFAULT_TIMESTAMP_MULTIPLIER = 1;
    
    /** default port for streaming AE Events from ARC smarteye TDS sensor */
    public static final int ARC_TDS_STREAM_PORT=20020;
    
    /** timestamp multiplier for ARC TDS smart eye sensor streaming data */
    public static final float ARC_TDS_TIMESTAMP_MULTIPLIER=0.001f; // TDS timestamps are 1ms
    
    /** ARC TDS smarteye swaps byte order since it comes from a non-intel system */
    public static final boolean ARC_TDS_SWAPBYTES_ENABLED=true;
    
    /** ARC TDS smarteye does not use sequence numbers */
    public static final boolean ARC_TDS_SEQUENCE_NUMBERS_ENABLED=false;
    
    /** ARC TDS smarteye sends address bytes first */
    public static final boolean ARC_TDS_ADDRESS_BYTES_FIRST_ENABLED=true;
    
    

    public boolean isSequenceNumberEnabled();

    /** If set true (default), then an int32 sequence number is the first word of the packet. Otherwise the
     * first int32 is part of the first AE. 
     * 
     * @param sequenceNumberEnabled default true
     */
    public void setSequenceNumberEnabled(boolean sequenceNumberEnabled);

    /** @see #setAddressFirstEnabled */
    public boolean isAddressFirstEnabled();

    /** If set true, the first int32 of each AE is the address, and the second is the timestamp. If false,
     * the first int32 is the timestamp, and the second is the address.
     * This parameter is stored as a preference.
     * @param addressFirstEnabled default true. 
     */
    public void setAddressFirstEnabled(boolean addressFirstEnabled);

    /** You need to setHost before this will receive events
    @param host the hostname
     */
    public void setHost(String host);

    public String getHost();

    public int getPort();

    public void setPort(int port);

    /** To handle big endian event sources/sinks 
     * (e.g. intel code) the address and timestamp bytes can be swapped from big to little endian format */
    public void setSwapBytesEnabled(boolean yes);

    public boolean isSwapBytesEnabled();

    public float getTimestampMultiplier();

    public void setTimestampMultiplier(float timestampMultiplier);
}
