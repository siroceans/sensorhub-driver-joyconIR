/***************************** BEGIN LICENSE BLOCK ***************************
 The contents of this file are subject to the Mozilla Public License, v. 2.0.
 If a copy of the MPL was not distributed with this file, You can obtain one
 at http://mozilla.org/MPL/2.0/.

 Software distributed under the License is distributed on an "AS IS" basis,
 WITHOUT WARRANTY OF ANY KIND, either express or implied. See the License
 for the specific language governing rights and limitations under the License.

 Copyright (C) 2020-2025 Botts Innovative Research, Inc. All Rights Reserved.
 ******************************* END LICENSE BLOCK ***************************/
package com.georobotix.impl.sensor.joyconIR;

// OSH imports
import org.sensorhub.api.common.SensorHubException;
import org.sensorhub.impl.sensor.AbstractSensorModule;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

// Functional Imports
import org.hid4java.*;
import java.io.IOException;
import java.util.Arrays;

/**
 * Driver implementation for the sensor.
 * <p>
 * This class is responsible for providing sensor information, managing output registration,
 * and performing initialization and shutdown for the driver and its outputs.
 */
public class JoyConImageSensor extends AbstractSensorModule<Config> {
    static final String UID_PREFIX = "urn:osh:joycon-image:";
    static final String XML_PREFIX = "JOYCON_IMAGE_";

    private static final Logger logger = LoggerFactory.getLogger(JoyConImageSensor.class);

    JoyConImageOutput output;
    Thread processingThread;
    volatile boolean doProcessing = true;

    // Functional variables.
    private HidDevice joycon;
    private int timingByte = 0;
    private int videoResolution = 240; // 240p
    private int irImageWidth;
    private int irImageHeight;
    private int irMaxFragNo;
    private byte irResReg;
    private final int numericIRExposure = 300; // Shutter speed in microseconds.
    private boolean enableVideo = false;
    private int vendorId = 0x057E;
    private int productId = 0x2007;
    private int resGet = 0;

    @Override
    public void doInit() throws SensorHubException, InterruptedException {
        super.doInit();

        // Joy Con constructor logic.
        HidServices services = HidManager.getHidServices(new HidServicesSpecification());
        this.joycon = services.getAttachedHidDevices().stream()
                .filter(d -> d.getVendorId() == vendorId && d.getProductId() == productId)
                .findFirst()
                .orElseThrow(() -> new HidException(String.format("No JoyCon found!!"))); // Store the joycon to device.
        joycon.open(); // Open the found Joycon.
        setResolution(videoResolution);

        // Joy Con Initialization Sequence
        silenceInputReport();
        setLedBusy();
        getSn((byte) 0x60, (byte) 0x01, 0x0F);
        getDeviceInfo();
        getBattery();
        getTemperature();
        getSpiData((byte) 0x60, (byte) 0x50, 12);
        sendRumble();

        // Generate identifiers
        generateUniqueID(UID_PREFIX, config.serialNumber);
        generateXmlID(XML_PREFIX, config.serialNumber);

        // Create and initialize output
        output = new JoyConImageOutput(this, irImageWidth, irImageHeight);
        addOutput(output, false);
        output.doInit();
    }

    @Override
    public void doStart() throws SensorHubException, IOException, InterruptedException {
        super.doStart();

        // Procedure before processing images! (9 steps)
        step0();
        step1();
        step2();
        step3();
        step4();
        step5();
        step6();
        step7();
        step8();

        startProcessing();
    }

    @Override
    public void doStop() throws SensorHubException, IOException {
        super.doStop();
        stopProcessing();
    }

    @Override
    public boolean isConnected() {
        return processingThread != null && processingThread.isAlive();
    }

    /**
     * Starts the data processing thread.
     * <p>
     * This method simulates sensor data collection and processing by generating data samples at regular intervals.
     */
    public void startProcessing() {
        doProcessing = true;

        processingThread = new Thread(() -> {
            while (doProcessing) {
                // Simulate data collection and processing -> moved to getRawIR Image
                //output.setData(System.currentTimeMillis(), "Sample Data");

                // Get Video Data
                try {
                    step9(true);
                } catch (IOException e) {
                    throw new RuntimeException(e);
                }
            }
        });
        processingThread.start();
    }

    /**
     * Signals the processing thread to stop.
     */
    public void stopProcessing() throws IOException {
        doProcessing = false;

        // stop processing images and end connection with the joycon.
        step9(false);
        close();
    }

    // ---------------------------------------------------------------------------------------------------------------//

    // Method to pick IR camera video resolution.
    private void setResolution(int irResolution) {
        if (irResolution == 240) { // I believe this is the native resolution of the sensor!
            this.irImageWidth = 320;
            this.irImageHeight = 240;
            this.irMaxFragNo = 0xFF;
            this.irResReg = 0x00; // Full pixel array.
        } else if(irResolution == 120) {
            this.irImageWidth = 160;
            this.irImageHeight = 120;
            this.irMaxFragNo = 0x3f;
            this.irResReg = 0b01010000; // Sensor binning [2x2]
        } else if(irResolution == 60) {
            this.irImageWidth = 80;
            this.irImageHeight = 60;
            this.irMaxFragNo = 0x0f;
            this.irResReg = 0b01100100; // Sensor binning [4x2] and skipping [1x2]
        } else if(irResolution == 30) {
            this.irImageWidth = 40;
            this.irImageHeight = 30;
            this.irMaxFragNo = 0x03;
            this.irResReg = 0b01101001; // Sensor binning [4x2] and skipping [2x4]
        }
    }

    // Method to send HID packets to joycon.
    private int send(byte[] packet, byte reportId) throws HidException {
        // Send a packet to the joycon
        return joycon.write(packet, packet.length, reportId);
    }

    // Method to receive HID packets from joycon.
    private int receive(byte[] buf, int timeoutMillis) throws HidException {
        // Receive a packet from the JoyCon.
        Arrays.fill(buf, (byte) 0);
        return joycon.read(buf, timeoutMillis);
    }

    // CLAMP function for integer values.
    public static int clampInt(int value, int low, int high) {
        if (value < low) return low;
        if (value > high) return high;
        return value;
    }

    // Method to perform CRC8 Calculation.
    private byte mcuCrc8Calc(byte[] buf,int size, int start) {
        byte crc8 = 0x00;
        final byte[] mcuCrc8Table = new byte[] {
                (byte)0x00, (byte)0x07, (byte)0x0E, (byte)0x09, (byte)0x1C, (byte)0x1B, (byte)0x12, (byte)0x15,
                (byte)0x38, (byte)0x3F, (byte)0x36, (byte)0x31, (byte)0x24, (byte)0x23, (byte)0x2A, (byte)0x2D,
                (byte)0x70, (byte)0x77, (byte)0x7E, (byte)0x79, (byte)0x6C, (byte)0x6B, (byte)0x62, (byte)0x65,
                (byte)0x48, (byte)0x4F, (byte)0x46, (byte)0x41, (byte)0x54, (byte)0x53, (byte)0x5A, (byte)0x5D,
                (byte)0xE0, (byte)0xE7, (byte)0xEE, (byte)0xE9, (byte)0xFC, (byte)0xFB, (byte)0xF2, (byte)0xF5,
                (byte)0xD8, (byte)0xDF, (byte)0xD6, (byte)0xD1, (byte)0xC4, (byte)0xC3, (byte)0xCA, (byte)0xCD,
                (byte)0x90, (byte)0x97, (byte)0x9E, (byte)0x99, (byte)0x8C, (byte)0x8B, (byte)0x82, (byte)0x85,
                (byte)0xA8, (byte)0xAF, (byte)0xA6, (byte)0xA1, (byte)0xB4, (byte)0xB3, (byte)0xBA, (byte)0xBD,
                (byte)0xC7, (byte)0xC0, (byte)0xC9, (byte)0xCE, (byte)0xDB, (byte)0xDC, (byte)0xD5, (byte)0xD2,
                (byte)0xFF, (byte)0xF8, (byte)0xF1, (byte)0xF6, (byte)0xE3, (byte)0xE4, (byte)0xED, (byte)0xEA,
                (byte)0xB7, (byte)0xB0, (byte)0xB9, (byte)0xBE, (byte)0xAB, (byte)0xAC, (byte)0xA5, (byte)0xA2,
                (byte)0x8F, (byte)0x88, (byte)0x81, (byte)0x86, (byte)0x93, (byte)0x94, (byte)0x9D, (byte)0x9A,
                (byte)0x27, (byte)0x20, (byte)0x29, (byte)0x2E, (byte)0x3B, (byte)0x3C, (byte)0x35, (byte)0x32,
                (byte)0x1F, (byte)0x18, (byte)0x11, (byte)0x16, (byte)0x03, (byte)0x04, (byte)0x0D, (byte)0x0A,
                (byte)0x57, (byte)0x50, (byte)0x59, (byte)0x5E, (byte)0x4B, (byte)0x4C, (byte)0x45, (byte)0x42,
                (byte)0x6F, (byte)0x68, (byte)0x61, (byte)0x66, (byte)0x73, (byte)0x74, (byte)0x7D, (byte)0x7A,
                (byte)0x89, (byte)0x8E, (byte)0x87, (byte)0x80, (byte)0x95, (byte)0x92, (byte)0x9B, (byte)0x9C,
                (byte)0xB1, (byte)0xB6, (byte)0xBF, (byte)0xB8, (byte)0xAD, (byte)0xAA, (byte)0xA3, (byte)0xA4,
                (byte)0xF9, (byte)0xFE, (byte)0xF7, (byte)0xF0, (byte)0xE5, (byte)0xE2, (byte)0xEB, (byte)0xEC,
                (byte)0xC1, (byte)0xC6, (byte)0xCF, (byte)0xC8, (byte)0xDD, (byte)0xDA, (byte)0xD3, (byte)0xD4,
                (byte)0x69, (byte)0x6E, (byte)0x67, (byte)0x60, (byte)0x75, (byte)0x72, (byte)0x7B, (byte)0x7C,
                (byte)0x51, (byte)0x56, (byte)0x5F, (byte)0x58, (byte)0x4D, (byte)0x4A, (byte)0x43, (byte)0x44,
                (byte)0x19, (byte)0x1E, (byte)0x17, (byte)0x10, (byte)0x05, (byte)0x02, (byte)0x0B, (byte)0x0C,
                (byte)0x21, (byte)0x26, (byte)0x2F, (byte)0x28, (byte)0x3D, (byte)0x3A, (byte)0x33, (byte)0x34,
                (byte)0x4E, (byte)0x49, (byte)0x40, (byte)0x47, (byte)0x52, (byte)0x55, (byte)0x5C, (byte)0x5B,
                (byte)0x76, (byte)0x71, (byte)0x78, (byte)0x7F, (byte)0x6A, (byte)0x6D, (byte)0x64, (byte)0x63,
                (byte)0x3E, (byte)0x39, (byte)0x30, (byte)0x37, (byte)0x22, (byte)0x25, (byte)0x2C, (byte)0x2B,
                (byte)0x06, (byte)0x01, (byte)0x08, (byte)0x0F, (byte)0x1A, (byte)0x1D, (byte)0x14, (byte)0x13,
                (byte)0xAE, (byte)0xA9, (byte)0xA0, (byte)0xA7, (byte)0xB2, (byte)0xB5, (byte)0xBC, (byte)0xBB,
                (byte)0x96, (byte)0x91, (byte)0x98, (byte)0x9F, (byte)0x8A, (byte)0x8D, (byte)0x84, (byte)0x83,
                (byte)0xDE, (byte)0xD9, (byte)0xD0, (byte)0xD7, (byte)0xC2, (byte)0xC5, (byte)0xCC, (byte)0xCB,
                (byte)0xE6, (byte)0xE1, (byte)0xE8, (byte)0xEF, (byte)0xFA, (byte)0xFD, (byte)0xF4, (byte)0xF3
        };

        for (int i = start-1; i < size + start -1; i++) {
            crc8 = mcuCrc8Table[(crc8 ^ buf[i]) & 0xff];
        }
        return crc8;
    }

    // Method to close the connection with the JoyCon.
    public void close() throws IOException {
        if (joycon != null && joycon.isOpen()) {
            step10();
            joycon.close();
        }
    }

    // ---------------------------------------------------------------------------------------------------------------//

    public void getRawIRImage() throws IOException {
        byte[] packet = new byte[48];
        byte[] bufImage = new byte[19*4096]; // 8bpp greyscale image.
        byte[] reply = new byte[0x170];

        /*
        int badSignal = 0;
        int errorReading = 0;
        float noiseLevel = 0.0f;
        int averageIntensityPercent = 0;
         */
        int previousFragNo = 0;
        int gotFragNo = 0;
        int missedPacketNo = 0;
        boolean missedPacket = false;
        int initialization = 2;
        int maxPixels = (217 + 1) * 300;
        int whitePixelsPercent = 0;
        boolean enableIRAutoExposure = true; // Remember the step in which we hardcoded this would always be true!
        int counter = 0;

        Arrays.fill(packet, (byte) 0);
        Arrays.fill(bufImage, (byte) 0);
        Arrays.fill(reply, (byte) 0);

        byte reportId = 0x11;
        packet[0] = (byte) (timingByte++ & 0xF);
        packet[9] = 0x03;
        packet[47] = (byte) 0xFF;

        // first ack
        packet[13] = 0x00;
        packet[46] = mcuCrc8Calc(packet, 36, 11);
        send(packet, reportId);

        // IR read/ack loop for fragmented data packets.
        // it also avoids requesting missed data fragments, we just skip it not to complicate things???
        while (this.enableVideo || initialization != 0) {
            Arrays.fill(reply, (byte) 0);
            receive(reply, 200);

            // Check if new packet.
            if ((reply[0] & 0xFF) == 0x31 &&
                    (reply[49] & 0xFF) == 0x03) {

                gotFragNo = Byte.toUnsignedInt(reply[52]);

                // Debug
                /*
                System.out.println("The value of reply[0] is: " + (reply[0] & 0xFF));
                System.out.println("The value of reply[49] is: " + (reply[49] & 0xFF));
                System.out.println("The value of reply[50] is: " + (reply[50] & 0xFF));
                System.out.println("The value of reply[51] is: " + (reply[51] & 0xFF));
                System.out.println("The value of reply[52] is: " + (reply[52] & 0xFF));
                System.out.println("The value of reply[53] is: " + (reply[53] & 0xFF));
                System.out.println("The value of reply[54] is: " + (reply[54] & 0xFF));
                System.out.println("The value of reply[55] is: " + (reply[55] & 0xFF));
                System.out.println("The value of reply[56] is: " + (reply[56] & 0xFF));
                System.out.println("Bytes 49 to 56 look as follows: ");

                for (int i = 49; i <= 56; i++) {
                    System.out.printf("reply[%d] = 0x%02X\n", i, reply[i] & 0xFF);
                }
                System.out.println("gotFragNo is: " + gotFragNo + " which can NOT be negative, or else the following if statement's logic fails");
                System.out.println("The value of irMaxFragNo is: " + irMaxFragNo);
                 */

                if (gotFragNo == (previousFragNo + 1) % (irMaxFragNo + 1)) {
                    // System.out.println("Went in 1");
                    previousFragNo = gotFragNo;

                    // ack for fragment.
                    packet[0] = (byte) (timingByte++ & 0xF);
                    packet[13] = (byte) previousFragNo;
                    packet[46] = mcuCrc8Calc(packet, 36, 11);
                    send(packet, reportId);
                    System.arraycopy(reply, 59, bufImage, 300 * gotFragNo, 300);

                    // Auto exposure.
                    if (enableIRAutoExposure == true && initialization < 2 && gotFragNo == 0) {
                        //System.out.println("Went in 2");
                        int whitePixels = ((reply[55] & 0xFF) | ((reply[56] & 0xFF) << 8)); // Little-endian u16
                        whitePixelsPercent = (whitePixels * 100) / maxPixels;
                        irSensorAutoExposure(whitePixelsPercent);
                    }

                    // Check if final fragment. Draw the frame..
                    if (gotFragNo == irMaxFragNo) {
                        /*
                        System.out.println("Went in 3");
                        noiseLevel = (float) (((reply[57] & 0xFF) | ((reply[58] & 0xFF) << 8)))
                                / ((float) (((reply[55] & 0xFF) | ((reply[56] & 0xFF) << 8))) + 1.0f);
                        whitePixelsPercent = (int) ((whitePixels * 100.0f) / maxPixels);
                        averageIntensityPercent = (int) ((reply[53] & 0xFF) * 100.0f / 255.0f);
                        int whitePixels = (reply[55] & 0xFF) | ((reply[56] & 0xFF) << 8);
                        String fileName = "frame_" + counter + ".png";
                         */

                        // Data collection and processing.
                        output.setData(System.currentTimeMillis(), bufImage);
                        counter++;

                        if (initialization != 0) {
                            initialization--;
                        }
                    }
                }

                // Repeat / Missed fragment
                else if (gotFragNo != 0 || previousFragNo != 0) {
                    // System.out.println("Went in 4");
                    // Check if repeat ACK should be sent. Avoid writing to image buffer.
                    if (gotFragNo  ==  previousFragNo) {
                        // System.out.println("Went in 5");

                        // ACK for fragment
                        packet[0] = (byte) (timingByte++ & 0xF);
                        packet[13] = (byte) gotFragNo;
                        packet[46] = mcuCrc8Calc(packet, 36, 11);
                        send(packet, reportId);

                        missedPacket = false;
                    }
                    // Check if missed fragment and request it.
                    else if(missedPacketNo != gotFragNo && !missedPacket) {
                        // System.out.println("Went in 6");
                        if (irMaxFragNo != 0x03) {
                            // System.out.println("Went in 7");

                            // Missed Packet
                            packet[0] = (byte) (timingByte++ & 0xF);
                            // Request for missed packet.You send what the next fragment number will be, instead of the acutal missed packet.
                            packet[11] = 0x01;
                            packet[12] = (byte) (previousFragNo + 1);
                            packet[13] = 0x00;
                            packet[46] = mcuCrc8Calc(packet, 36, 11);
                            send(packet, reportId);

                            packet[11] = 0x00;
                            packet[12] = 0x00;

                            System.arraycopy(reply, 59, bufImage, 300 * gotFragNo, 300);

                            previousFragNo = gotFragNo;
                            missedPacketNo = gotFragNo - 1;
                            missedPacket = true;
                        }
                        // Check if missed fragment and res is 30x40. Don't Request it.
                        else {
                            // System.out.println("Went in 8");
                            // ack for fragment
                            packet[0] = (byte) (timingByte++ & 0xF);
                            packet[13] = (byte) gotFragNo;
                            packet[46] = mcuCrc8Calc(packet, 36, 11);
                            send(packet, reportId);

                            System.arraycopy(reply, 59, bufImage, 300 * gotFragNo, 300);
                            previousFragNo = gotFragNo;
                        }
                    }
                    // Got the requested missing fragments.
                    else if (missedPacketNo == gotFragNo) {
                        // System.out.println("Went in 9");
                        // ack for fragment
                        packet[0] = (byte) (timingByte++ & 0xF);
                        packet[13] = (byte) gotFragNo;
                        packet[46] = mcuCrc8Calc(packet, 36, 11);
                        send(packet, reportId);

                        System.arraycopy(reply, 59, bufImage, 300 * gotFragNo, 300);
                        previousFragNo = gotFragNo;
                        missedPacket = false;
                    }
                    // Repeat of fragment that is not max fragment.
                    else {
                        // System.out.println("Went in 10");
                        // ack for fragment.
                        packet[0] = (byte) (timingByte++ & 0xF);
                        packet[13] = (byte) gotFragNo;
                        packet[46] = mcuCrc8Calc(packet, 36, 11);
                        send(packet, reportId);
                    }
                }

                // Streaming Start
                else {
                    // System.out.println("Went in 11");
                    packet[0] = (byte) (timingByte++ & 0xF);
                    packet[13] = (byte) gotFragNo;
                    packet[46] = mcuCrc8Calc(packet, 36, 11);
                    send(packet, reportId);

                    System.arraycopy(reply, 59, bufImage, 300 * gotFragNo, 300);
                    previousFragNo = 0;
                }
            }

            // Empty IR Report. Send Ack again. Otherwise it fallbacks to high latency mode (30 ms per data fragment)
            else if ((reply[0] & 0xFF) == 0x31) {
                // System.out.println("Went in 12, sending ack again.");
                // ack for fragment.
                packet[0] = (byte) (timingByte++ & 0xF);

                /*
                // Send ack again or request missed frag
                if ((reply[49] & 0xFF) == 0xFF) {
                    System.out.println("Went in 13");
                    packet[13] = (byte) previousFragNo;
                }
                else if((reply[49] & 0xFF) == 0x00) {
                    System.out.println("Went in 14");
                    packet[11] = 0x01;
                    packet[12] = (byte) (previousFragNo + 1);
                    packet[13] = 0x00;
                }
                */

                // System.out.println("Went in 15");
                packet[46] = mcuCrc8Calc(packet, 36, 11);
                send(packet, reportId);

                packet[11] = 0x00;
                packet[12] = 0x00;
                //debug
                packet[13] = 0x00;
            }
        }
    }

    public void irSensorAutoExposure(int whitePixelsPercent) throws IOException {
        int res;
        byte[] packet = new byte[48];
        Arrays.fill(packet, (byte) 0);
        int newExposure = 0;
        int oldExposure = numericIRExposure;

        // Calcuate the new exposure!
        if (whitePixelsPercent == 0) {
            oldExposure += 10;
        } else if(whitePixelsPercent > 5) {
            oldExposure -= (whitePixelsPercent / 4) * 20;
        }

        oldExposure = clampInt(oldExposure, 0, 600);
        newExposure = oldExposure * 31200 / 1000;

        byte reportId = 0x01;
        packet[0] = (byte) (timingByte++ & 0xF);
        packet[9] = 0x21;

        packet[10] = 0x23; // Write register cmd
        packet[11] = 0x04; // Write register to IR mode subcmd
        packet[12] = 0x03; // Number of registers to write
        // Register 1
        packet[13] = 0x01;
        packet[14] = 0x30; // R: 0x0130 - Set exposure time LSByte.
        packet[15] = (byte) (newExposure & 0xFF);
        // Register 2
        packet[16] = 0x01;
        packet[17] = 0x31; // R: 0x0131 - Set exposure time MSByte.
        packet[18] = (byte) ((newExposure & 0xFF00) >> 8);
        // Register 3
        packet[19] = 0x00;
        packet[20] = 0x07; // R: 0x0007 - Finalize config - without this, the register changes do not take any effect.
        packet[21] = 0x01;
        // Calculate CRC8
        packet[47] = mcuCrc8Calc(packet, 36, 12);
        send(packet, reportId);
    }

    private int silenceInputReport() {
        int res;
        byte[] packet = new byte[48];
        Arrays.fill(packet, (byte) 0);

        for (int error = 0; error < 6; error++) {
            byte reportId = 0x01;
            packet[0] = (byte) (timingByte++ & 0xF);
            packet[9] = 0x03;
            packet[10] = 0x3f;
            res = send(packet, reportId);
            for (int i = 0; i < 9; i++) {
                byte[] reply = new byte[49];
                res = receive(reply, 64);
                if ((reply[13] & 0xFF) == 0x80 &&
                        (reply[14] & 0xFF) == 0x03) {
                    return 0;
                }
            }
        }
        return 1;
    }

    private int setLedBusy() {
        int res;
        byte[] packet = new byte[48];
        Arrays.fill(packet, (byte) 0);
        byte[] reply = new byte[49];

        byte reportId = 0x01;
        packet[0] = (byte) (timingByte++ & 0xFF);
        packet[9] = 0x30;
        packet[10] = (byte) 0x81;
        res = send(packet, reportId);
        res = receive(reply, 64);

        // Set breathing HOME LED
        Arrays.fill(packet, (byte) 0);
        Arrays.fill(reply, (byte) 0);
        byte reportId2 = 0x01;
        packet[0] = (byte) (timingByte++ & 0xF);
        packet[9] = 0x38;
        packet[10] = 0x28;
        packet[11] = 0x20;
        packet[13] = (byte) 0xF2;
        packet[14] = (byte) 0xF0;
        packet[15] = (byte) 0xF0;

        res = send(packet, reportId2);
        res = receive(reply, 64);

        return 0;
    }

    private String getSn(byte byte2, byte byte1, int read_len) {
        int res;
        byte[] packet = new byte[48];
        String test = "";

        for (int error = 0; error < 21; error++) {
            Arrays.fill(packet, (byte) 0);
            byte reportId = 0x01;
            packet[0] = (byte) (timingByte++ & 0xF);
            packet[9] = 0x10;
            packet[10] = byte1;
            packet[11] = byte2;
            packet[14] = (byte) read_len;
            res = send(packet, reportId);

            for (int i = 0; i < 9; i++) {
                byte[] reply = new byte[49];
                res = receive(reply, 64);
                if ((reply[13] & 0xFF) == 0x90 &&
                        (reply[14] & 0xFF) == 0x10) {
                    //check_result
                    if (res >= 0x14 + read_len) {
                        for (int x = 0; x < read_len; x++) {
                            if ((reply[20 + x] & 0xFF) != 0x00) {
                                test += (char) (reply[20 + x] & 0xFF);
                            } else {
                                test += "";
                            }
                        }
                        return test;
                    }
                } else {
                    return "Error in getSn()";
                }
            }
        }
        return "Error!";
    }

    private byte[] getDeviceInfo() {
        // Did not include test_buf, i dont think its necessary??
        int res;
        byte[] packet = new byte[48];
        byte[] deviceInfo = new byte[10];

        for (int error = 0; error < 21; error++) {
            Arrays.fill(packet, (byte) 0);
            byte reportId = 0x01;
            packet[0] = (byte) (timingByte++ & 0xF);
            packet[9] = 0x02;
            res = send(packet, reportId);

            for (int i = 0; i < 9; i++) {
                byte[] reply = new byte[49];
                res = receive(reply, 64);
                if ((reply[13] & 0xFF) == 0x82 &&
                        (reply[14] & 0xFF) == 0x02) {
                    //Check result
                    for (int x = 0; x < 10; x++) {
                        deviceInfo[i] = reply[15 + i];
                    }
                    System.out.println("getDeviceInfo successful");
                    return deviceInfo;
                }
            }
        }
        System.err.println("getDeviceInfo failed. ");
        return deviceInfo;
    }

    private byte[] getBattery() {
        byte[] battInfo = new byte[3];
        Arrays.fill(battInfo, (byte) 0);
        byte[] packet = new byte[48];
        int res;

        for (int error = 0; error < 21; error++) {
            Arrays.fill(packet, (byte) 0);
            byte reportId = 0x01;
            packet[0] = (byte) (timingByte++ & 0xF);
            packet[9] = 0x50;
            res = send(packet, reportId);

            for (int i = 0; i < 9; i++) {
                byte[] reply  = new byte[49];
                res = receive(reply, 64);
                if ((reply[13] & 0xFF) == 0xD0 &&
                        (reply[14] & 0xFF) == 0x50) {
                    //check_result
                    battInfo[0] = reply[0x02];
                    battInfo[1] = reply[0x0F];
                    battInfo[2] = reply[0x10];

                    System.out.println("getBattery Succesful!");
                    return battInfo;
                }
            }
        }
        System.err.println("getBattery failed. ");
        return battInfo;
    }

    private byte[] getTemperature() throws InterruptedException{
        byte[] tempInfo = new byte[2];
        int res;
        byte[] packet = new byte[48];
        boolean imuChanged = false;

        for (int error = 0; error < 21; error++) {
            Arrays.fill(packet, (byte) 0);
            byte reportId = 0x01;
            packet[0] = (byte) (timingByte++ & 0xF);
            packet[9] = 0x43;
            packet[10] = 0x10;
            packet[11] = 0x01;
            res = send(packet, reportId);

            for (int i = 0; i < 9; i++) {
                byte[] reply = new byte[49];
                res = receive(reply, 64);
                if ((reply[13] & 0xFF) == 0xC0 &&
                        (reply[14] & 0xFF) == 0x43) {
                    //check_result
                    if ((reply[0x11] >> 4) == 0x00) {
                        Arrays.fill(packet, (byte) 0);
                        reportId = 0x01;
                        packet[0] = (byte) (timingByte++ & 0xF);
                        packet[9] = 0x40;
                        packet[10] = 0x01;
                        res = send(packet, reportId);
                        res = receive(reply, 64);

                        imuChanged = true;

                        // We let the temperature sensor stabilize for a bit.
                        Thread.sleep(64);
                    }
                    for (int error2 = 0; error2 < 21; error2++) {
                        Arrays.fill(packet, (byte) 0);
                        reportId = 0x01;
                        packet[0] = (byte) (timingByte++ & 0xF);
                        packet[9] = 0x43;
                        packet[10] = 0x20;
                        packet[11] = 0x02;
                        res = send(packet, reportId);

                        for (int i2 = 0; i2 < 9; i2++) {
                            res = receive(reply, 64);
                            if ((reply[13] & 0xFF) == 0xC0 &&
                                    (reply[14] & 0xFF) == 0x43) {
                                //check_result2
                                tempInfo[0] = reply[0x11];
                                tempInfo[1] = reply[0x12];

                                if (imuChanged) {
                                    Arrays.fill(packet, (byte) 0);
                                    reportId = 0x01;
                                    packet[0] = (byte) (timingByte++ & 0xF);
                                    packet[9] = 0x40;
                                    packet[10] = 0x00;
                                    res = send(packet, reportId);
                                    res = receive(reply, 64);
                                }
                                System.out.println("getTemperature Succesful. ");
                                return tempInfo;
                            }
                        }
                    }
                }
            }
        }
        System.err.println("getTemperature Failed.");
        return tempInfo;
    }

    private byte[] getSpiData(byte byte2, byte byte1, int readLen) {
        int res;
        byte[] spiColors = new byte[12];
        byte[] packet = new byte[48];

        for (int error = 0; error < 21; error++) {
            Arrays.fill(packet, (byte) 0);
            byte reportId = 0x01;
            packet[0] = (byte) (timingByte++ & 0xF);
            packet[9] = 0x10;
            packet[10] = byte1;
            packet[11] = byte2;
            packet[14] = (byte) readLen;
            res = send(packet, reportId);

            for (int i = 0; i < 9; i++) {
                byte[] reply = new byte[49];
                res = receive(reply, 64);
                int extractedOffset = (reply[15] & 0xFF) |
                        ((reply[16] & 0xFF) << 8) |
                        ((reply[17] & 0xFF) << 16) |
                        ((reply[18] & 0xFF) << 24);
                int offset = ((byte2 & 0xFF) << 8) | (byte1 & 0xFF);  // Little endian
                if ((reply[13] & 0xFF) == 0x90 &&
                        (reply[14] & 0xFF) == 0x10 &&
                        (extractedOffset == offset)) {
                    //check_result.
                    if (res >= 0x14 + readLen) {
                        for (int x = 0; x < readLen; x++) {
                            spiColors[i] = reply[0x14 + x];
                        }
                        System.out.println("getSpiData successful.");
                        return spiColors;
                    }
                }
            }
        }
        System.err.println("getSpiData failed.");
        return spiColors;
    }

    private int sendRumble() throws InterruptedException{
        int res;
        byte[] buf = new byte[48];
        byte[] buf2 = new byte[49];

        // Enable Vibration.
        Arrays.fill(buf, (byte) 0);
        byte reportId = 0x01;
        buf[0] = (byte) (timingByte++ & 0xF);
        buf[9] = 0x48;
        buf[10] = 0x01;
        res = send(buf, reportId);
        res = receive(buf2, 64);

        // New vibration like switch.
        Thread.sleep(16);
        //Send confirmation
        Arrays.fill(buf, (byte) 0);
        reportId = 0x01;
        buf[0] = (byte) (timingByte++ & 0xF);
        buf[1] = (byte) 0xC2;
        buf[2] = (byte) 0xC8;
        buf[3] = 0x03;
        buf[4] = 0x72;
        buf[5] = (byte) 0xC2;
        buf[6] = (byte) 0xC8;
        buf[7] = 0x03;
        buf[8] = 0x72;
        res = send(buf, reportId);
        res = receive(buf2, 64);

        Thread.sleep(81);
        buf[0] = (byte) (timingByte++ & 0xF);
        buf[1] = 0x00;
        buf[2] = 0x01;
        buf[3] = 0x40;
        buf[4] = 0x40;
        buf[5] = 0x00;
        buf[6] = 0x01;
        buf[7] = 0x40;
        buf[8] = 0x40;
        res = send(buf, reportId);
        res = receive(buf2, 64);

        Thread.sleep(5);
        buf[0] = (byte) (timingByte++ & 0xF);
        buf[1] = (byte) 0xC3;
        buf[2] = (byte) 0xC8;
        buf[3] = 0x60;
        buf[4] = 0x64;
        buf[5] = (byte) 0xC3;
        buf[6] = (byte) 0xC8;
        buf[7] = 0x60;
        buf[8] = 0x64;
        res = send(buf, reportId);
        res = receive(buf2, 64);

        Thread.sleep(5);

        // Disable Vibration
        Arrays.fill(buf, (byte) 0);
        reportId = 0x01;
        buf[0] = (byte) (timingByte++ & 0xF);
        buf[1] = 0x00;
        buf[2] = 0x01;
        buf[3] = 0x40;
        buf[4] = 0x40;
        buf[5] = 0x00;
        buf[6] = 0x01;
        buf[7] = 0x40;
        buf[8] = 0x40;
        buf[9] = 0x48;
        buf[10] = 0x00;
        res = send(buf, reportId);
        res = receive(buf2, 64);

        Arrays.fill(buf, (byte) 0);
        reportId = 0x01;
        buf[0] = (byte) (timingByte++ & 0xF);
        buf[9] = 0x30;
        buf[10] = 0x01;
        res = send(buf, reportId);
        res = receive(buf2, 64);

        // set HOME Led!! (the most exciting part :D)
        Arrays.fill(buf, (byte) 0);
        reportId = 0x01;
        buf[0] = (byte) (timingByte++ & 0xF);
        buf[9] = 0x38;
        // Heartbeat style config!!
        buf[10] = (byte) 0xF1;
        buf[11] = 0x00;
        buf[12] = buf[13] = buf[14] = buf[15] = buf[16] = buf[17] = (byte) 0xF0;
        buf[18] = buf[21] = buf[24] = buf[27] = buf[30] = 0x00;
        buf[19] = buf[20] = buf[22] = buf[23] = buf[25] = buf[26] = buf[28] = buf[29] = buf[31] = buf[32] = (byte) 0xFF;
        res = send(buf, reportId);
        res = receive(buf2, 64);

        return 0;
    }

    // ---------------------------------------------------------------------------------------------------//

    // STEP 0: Switch the controller's Input Report to 0x31. !
    public boolean step0() throws IOException, InterruptedException {
        for (int error = 0; error < 8; error ++) {
            int written;
            int res;

            // Switches the Joycon into IR mode (by sending subcommand 0x03 with argument 0x31)
            byte[] packet = new byte[48];  // Create 49 byte packet (standard joycon command size)
            Arrays.fill(packet, (byte) 0); // Fill it with 0's

            byte reportId = 0x01; // BCRM vendor command.
            packet[0] = (byte) (timingByte++ & 0xF); // Remember that (byte) is used to cast int to byte.
            packet[9] = 0x03; // Subcommand 0x03 = switch mode.
            packet[10] = 0x31; // Mode: set Incident Report Mode.
            written = send(packet, reportId); // Send the packet to the JoyCon.

            /* Debug
            System.out.println("The timing Byte is: " + packet[1]);
            System.out.println("step 01: hid_write returned : " + written);
            */

            Thread.sleep(50); // Letting joycon catch up
            for (int i = 0; i < 9; i++) {
                // Try this 8 times to get a valid reply.
                byte[] reply = new byte[0x170];
                res = receive(reply, 64);

                /* Debug
                System.out.println("Reply ["+i+"]: length = "+ reply.length);
                System.out.println("Byte 14 is: " + reply[14]);
                System.out.println("Byte 13 is: " + reply[13]);
                System.out.println("Byte 12 is: " + reply[12]);
                System.out.println("Byte 11 is: " + reply[11]);
                System.out.println("Byte 10 is: " + reply[10]);
                */

                if ((reply[13] & 0xFF) == 0x80 &&
                        (reply[14] & 0xFF) == 0x03) {
                    // Byte 13 should be 0x80 and Byte 14 should be 0x03.
                    System.out.println("Step 0 Successful.");
                    return true;
                }
            }
        }
        // If we get here, then we failed to switch to IR mode.
        System.err.println("Step 0 Failed.");
        resGet = 1;
        return false;
    }

    // STEP 1: Enable the MCU --- !
    public boolean step1() throws IOException {
        for (int error = 0; error < 9; error++) {
            int res;

            byte[] packet = new byte[48];
            Arrays.fill(packet, (byte) 0);

            // Now we give subcommand 0x22 with argument 0x01 to enable the MCU.
            byte reportId = 0x01;
            packet[0] = (byte) (timingByte++ & 0xF);
            packet[9] = 0x22;  // Subcommand 0x22 = MCU control.
            packet[10] = 0x01;  // Argument 0x01 = Enable MCU.
            send(packet, reportId);

            for (int i = 0; i < 8; i++) {
                byte[] reply = new byte[0x170];
                res = receive(reply, 64);
                if ((reply[13] & 0xFF) == 0x80 &&
                        (reply[14] & 0xFF) == 0x22) {
                    System.out.println("Step 1 Successful. :D");
                    return true;
                }
            }
        }
        // If it didn't manage to enable the MCU...
        System.err.println("Step 1 Failed.");
        resGet = 2;
        return false;
    }

    // STEP 2: Request MCU status until "Standby" --- !
    public boolean step2() throws IOException {
        int res;

        // Repeatedly sends subcommand 0x11 and checks the return to see if the MCU is in standby mode.
        for (int error = 0; error <= 8; error++) {
            byte[] packet = new byte[48];
            Arrays.fill(packet, (byte) 0);

            byte reportId = 0x11; // Output report instead of subcommand report. Check notes to see the difference on them.
            packet[0] = (byte) (timingByte++ & 0xF);
            packet[9] = 0x01; // Subcommand 0x01 = Get MCU status.
            send(packet, reportId);

            for (int i = 0; i < 9; i++) {
                byte[] reply = new byte[0x170];
                res = receive(reply, 64);
                if (reply[0] == 0x31 &&
                        reply[49] == 0x01 &&
                        reply[56] == 0x01) {
                    // IF byte56 == 1 -> "Standby state"
                    System.out.println("Step 2 Successful. :D");
                    return true;
                }
            }
        }
        // If we get to this point, step 02 has failed.
        System.err.println("Step 2 Failed.");
        resGet = 3;
        return false;
    }

    // STEP 3:  Set MCU Mode ---
    public boolean step3() throws IOException {
        int res;

        for (int error = 0; error < 9; error++) {
            byte[] packet = new byte[48];
            Arrays.fill(packet, (byte) 0);

            byte reportId = 0x01;
            packet[0] = (byte) (timingByte++ & 0xF);
            packet[9] = 0x21;
            packet[10] = 0x21; //set MCU mode cmd.
            packet[11] = 0x00; // set MCU mode cmd.
            packet[12] = 0x05; // MCU Mode - 1: Standby, 4: NFC, 5: IR, 6:???
            packet[47] = mcuCrc8Calc(packet, 36, 12);
            send(packet, reportId);

            for (int i = 0; i < 9; i++) {
                byte[] reply = new byte[0x170];
                res = receive(reply, 64);
                int mcuStatus = ((reply[25] & 0xFF) << 24) |
                        ((reply[24] & 0xFF) << 16) |
                        ((reply[23] & 0xFF) << 8) |
                        (reply[22] & 0xFF);
                if ((reply[0] & 0xFF) == 0x21 &&
                        (reply[15] & 0xFF) == 0x01 &&
                        mcuStatus == 0x01) {
                    System.out.println("Step 3 Successful. :D");
                    return true;
                }
            }
        }
        // if we couldn't enable it...
        System.err.println("Step 3 Failed.");
        resGet = 4;
        return false;
    }

    // STEP 4: Request MCU Mode Status.
    public boolean step4() throws IOException {
        int res;

        for (int error = 0 ; error < 8; error++) {
            byte[] packet = new byte[48];
            Arrays.fill(packet, (byte) 0);

            byte reportId = 0x11; // Remember!! 0x11 = output report.
            packet[0] = (byte) (timingByte++ & 0xF);
            packet[9] = 0x01; // Get status
            send(packet, reportId);

            for (int i = 0; i < 9; i++){
                byte[] reply = new byte[0x170];
                res = receive(reply, 64);
                if ((reply[0] & 0xFF) == 0x31 &&
                        (reply[49] & 0xFF) == 0x01 &&
                        (reply[56] & 0xFF) == 0x05) {
                    System.out.println("Step 4 successful. :D");
                    return true;
                }
            }
        }
        System.err.println("Step 4 failed.");
        resGet = 5;
        return false;
    }

    // STEP 5: Set IR mode and number of packets for each data blob. Blob size is packets * 300 bytes.
    public boolean step5() throws IOException {
        int res;

        for (int error = 0; error < 8; error++) {
            byte[] packet = new byte[48];
            Arrays.fill(packet, (byte) 0);

            byte reportId = 0x01;
            packet[0] = (byte) (timingByte++ & 0xF);
            packet[9] = 0x21; // subcommand: set exposure/timing
            packet[10] = 0x23;
            packet[11] = 0x01; // set IR mode command.
            packet[12] = 0x07; // IR Mode - 7: Image transfer. There are more modes!!
            packet[13] = (byte) irMaxFragNo;
            packet[14] = 0x00;
            packet[15] = 0x05; // setting IR MCU major
            packet[16] = 0x00;
            packet[17] = 0x18; // setting IR MCU minor
            packet[47] = mcuCrc8Calc(packet, 36, 12);
            send(packet, reportId);

            for (int i = 0; i < 8; i++) {
                byte[] reply = new byte[0x170];
                res = receive(reply, 64);
                if ((reply[0] & 0xFF) == 0x21 &&
                        (reply[15] & 0xFF) == 0x0B) {
                    System.out.println("Step 5 successful. :D");
                    return true;
                }
            }
        }
        System.err.println("Error in Step 5.");
        resGet = 6;
        return false;
    }

    // STEP 6: Request the IR mode status.
    public boolean step6() throws IOException {
        int res;

        for (int error = 0; error < 8; error++) {
            // debugging -- mistery packet!
            byte[] packet = new byte[48];
            Arrays.fill(packet, (byte) 0);
            byte reportId0 = 0x01;
            packet[0] = (byte) (timingByte++ & 0xF);
            packet[9] = 0x21;
            packet[10] = 0x23;
            packet[11] = 0x04;
            packet[12] = 0x09;
            packet[13] = 0x00;
            packet[14] = 0x2e;
            packet[15] = 0x00;
            packet[16] = 0x01;
            packet[17] = 0x30;
            packet[18] = (byte) 0x90;
            packet[19] = 0x01;
            packet[20] = 0x31;
            packet[21] = 0x24;
            packet[22] = 0x01;
            packet[23] = 0x32;
            packet[24] = 0x00;
            packet[25] = 0x00;
            packet[26] = 0x10;
            packet[27] = 0x00;
            packet[28] = 0x01;
            packet[29] = 0x2e;
            packet[30] = 0x20;
            packet[31] = 0x01;
            packet[32] = 0x2f;
            packet[33] = 0x00;
            packet[34] = 0x00;
            packet[35] = 0x0e;
            packet[36] = 0x03;
            packet[37] = 0x01;
            packet[38] = 0x43;
            packet[39] = (byte) 0xc8;
            packet[47] = mcuCrc8Calc(packet, 36, 11);
            send(packet, reportId0);

            Arrays.fill(packet, (byte) 0);

            byte reportId = 0x11;
            packet[0] = (byte) (timingByte++ & 0xF);
            packet[9] = 0x03;
            packet[10] = 0x02;
            packet[46] = mcuCrc8Calc(packet, 36, 11);
            packet[47] = (byte) 0xFF;
            send(packet, reportId);

            for (int i = 0; i < 5; i++) {
                byte[] reply = new byte[0x170];
                res = receive(reply, 64);
                if ((reply[0] & 0xFF) == 0x31 &&
                        (reply[49] & 0xFF) == 0x13 &&
                        (reply[50] & 0xFF) == 0x00 &&
                        (reply[51] & 0xFF) == 0x07) {
                    System.out.println("Step 6 successful!");
                    return true;
                }
            }
        }
        System.err.println("Error in Step 6.");
        resGet = 7;
        return false;
    }

    // STEP 7: Write to registers for the selected IR mode.
    public boolean step7() throws IOException {
        int res;
        byte irDigitalGain = 0x01;
        byte irExLightFilter = 0x03;
        byte irLeds = 0b00000001;
        int irExposure = numericIRExposure * 31200 / 1000;

        for (int error = 0; error < 8; error++) {
            byte[] packet = new byte[48];
            Arrays.fill(packet, (byte) 0);

            byte reportId = 0x01;
            packet[0] = (byte) (timingByte++ & 0xF);
            packet[9] = 0x21;

            packet[10] = 0x23; // Write register cmd
            packet[11] = 0x04; // Write register to IR mode subcmd.
            packet[12] = 0x09; // Number of registers to write.. Max 9.
            // Register 1
            packet[13] = 0x00;
            packet[14] = 0x2E; // R: 0x002E - set resolution based on sensor binning and skipping??????
            packet[15] = irResReg; // binning and skipping!
            // Register 2
            packet[16] = 0x01;
            packet[17] = 0x30; // R: 0x0130 - set exposure time.
            // debugging!! packet[18] = (byte) (irExposure & 0xFF); // Calculation of actual exposure value.
            packet[18] = (byte) 0x90;
            // Register 3
            packet[19] = 0x01;
            packet[20] = 0x31; // R: 0x0131 - set exposure time MSByte - ((31200 * us / 1000) & 0xFF00) >> 8
            // debugging! packet[21] = (byte) ((irExposure & 0xFF00) >> 8);
            packet[21] = 0x24;
            // Register 4
            packet[22] = 0x01;
            packet[23] = 0x32; // R: 0x0132 - eable max exposure time - 0: manual exposure, 1: max exposure.
            packet[24] = 0x00;
            // Register 5
            packet[25] = 0x00;
            packet[26] = 0x10; // R: 0x0010 - set ir leds groups state - only 3 lsb usable??
            // debugging! packet[27] = irLeds; // 0b00000001 - enable both LED groups w/ flashlight mode
            packet[27] = 0x00;
            // Register 6
            packet[28] = 0x01;
            packet[29] = 0x2E; // R 0x012E - Set digital gain LSH 4 bits of the value - 0-0xff
            packet[30] = 0x20; // debugging! (byte) ((irDigitalGain & 0x0F) << 4); // digital Gain = 0x01 - Disbable gain, enable auto exposure ------------------------------[]
            // Register 7
            packet[31] = 0x01;
            packet[32] = 0x2F; // R: 0x012D - set digital gain MSB 4 bits of the value - 0x07.
            packet[33] = 0x00; // debugging! (byte) ((irDigitalGain & 0xF0) >> 4); // value of reg 7
            // Register 8
            packet[34] = 0x00;
            packet[35] = 0x0E; // R: 0x00E0 - External light filter - LS o bit0: off/on, bit1: 0x/1x, bit2: ??, bit4,5: ??
            packet[36] = irExLightFilter; // 0x03 - filtering enabled, 0x00 - filtering disabled.
            // Register 9
            packet[37] = 0x01;
            packet[38] = 0x43; // R: 0x0143 - ExLF/White pixel stats threshold - 200: Default
            packet[39] = (byte) 0xC8;
            // Calculate CRC8
            packet[47] = mcuCrc8Calc(packet, 36, 12);
            send(packet, reportId);

            // Check for IR mode status: Mode 7 - Image transfer mode.
            byte[] packet2 = new byte[48];
            Arrays.fill(packet2, (byte) 0);

            byte reportId2 = 0x11;
            packet2[0] = (byte) (timingByte++ & 0xF);

            packet2[9] = 0x03;
            packet2[10] = 0x02;
            packet2[46] = mcuCrc8Calc(packet2, 36, 11);
            packet2[47] = (byte) 0xFF;
            send(packet2, reportId2);

            for (int i = 0; i < 9; i++) {
                byte[] reply = new byte[0x170];
                res = receive(reply, 64);
                if ((reply[0] & 0xFF) == 0x21 &&
                        (reply[15] & 0xFF) == 0x13 &&
                        (reply[16] & 0xFF) == 0x00 &&
                        (reply[17] & 0xFF) == 0x07) {
                    System.out.println("Step 7 successful.:D");
                    return true;
                }
            }
        }
        System.err.println("Error in Step 7.");
        resGet = 8;
        return false;
    }

    // STEP 8: Write to registers for the selected IR mode.
    public boolean step8() throws IOException {
        int res;

        for (int error = 0; error < 8; error++) {
            byte[] packet = new byte[48];
            Arrays.fill(packet, (byte) 0);

            byte reportId = 0x01;
            packet[0] = (byte) (timingByte++ & 0xF);

            packet[9] = 0x21;
            packet[10] = 0x23; // Write register cmd.
            packet[11] = 0x04; // Write register to IR mode subcmd.
            packet[12] = 0x08; // Number of registers to write.. Max 9
            // Register 1
            packet[13] = 0x00;
            packet[14] = 0x11; // R: 0x0011 - Leds 1/2 Intensity - Max 0x0F.
            packet[15] = (byte) 0x0F; // 0x0F - Max Intensity.
            // Register 2
            packet[16] = 0x00;
            packet[17] = 0x12; // R: 0x0012 - Leds 3/4 Intensity - Max 0x10
            packet[18] = (byte) 0x10; // Max intensity
            // Register 3
            packet[19] = 0x00;
            packet[20] = 0x2D; // R: 0x002D - Flip image - 0: Normal, 1: Vertically, 2: Horizontally, 3: both.
            packet[21] = 0x00; // 0 - normal.
            // Register 4
            packet[22] = 0x01;
            packet[23] = 0x67; // R: 0x0167 - Enable de-noise smoothing algorithms - 0: Disable, 1: Enable.
            packet[24] = 0x01; // Enabled
            // Register 5
            packet[25] = 0x01;
            packet[26] = 0x68; // R: 0x0168 - Edge smoothing threshold - Max 0xFF, Default 0x23
            packet[27] = 0x23; // Default Edge Smoothing Value.
            // Register 6
            packet[28] = 0x01;
            packet[29] = 0x69; // R: 0x0169 - Color Interpolation threshold - Max 0xFF, Default 0x44.
            packet[30] = 0x44; // Default color interpolation value.
            // Register 7
            packet[31] = 0x00;
            packet[32] = 0x04; // R: 0x0400 - LSB Buffer Update Time - Default 0x32
            packet[33] = 0x32; // Check if statement in cpp to look for the exception in here ----------------------------[]
            // Register 8
            packet[34] = 0x00;
            packet[35] = 0x07; // R: 0x0007 - Finalize config - Without this, the register changes do not have any effect!!
            packet[36] = 0x01;
            // Calculate CRC8
            packet[47] = mcuCrc8Calc(packet, 36, 12);
            send(packet, reportId);

            for (int i = 0; i < 8; i++) {
                byte[] reply = new byte[0x170];
                res = receive(reply, 64);
                if ((reply[0] & 0xFF) == 0x21 &&
                        (reply[15] & 0xFF) == 0x13 &&
                        (reply[16] & 0xFF) == 0x00 &&
                        (reply[17] & 0xFF) == 0x07) {
                    System.out.println("Step 8 was successful.:D");
                    return true;
                }
            }
        }
        System.err.println("Step 8 failed.");
        resGet = 9;
        return false;
    }

    // STEP 9: Stream or capture images from NIR Camera
    public boolean step9(boolean onOff) throws IOException {
        if (onOff) {
            this.enableVideo = true; // Enable video streaming
            getRawIRImage(); // Start streaming the video
            System.out.println("Step 9 successful... Streaming Video :D");
            return true;
        }
        else {
            this.enableVideo = false; // Stop Streaming
            System.out.println("Video Streaming Stopped...");
            return false;
        }
    }

    // STEP 10: Disable the MCU (to end the communication??)
    public boolean step10() throws IOException {
        int res;
        byte[] packet = new byte[48];
        Arrays.fill(packet, (byte) 0);

        byte reportId = 0x01;
        packet[0] = (byte) (timingByte++ & 0xF);

        packet[9] = 0x03;
        packet[10] = 0x3F;
        send(packet, reportId);

        for (int i = 0; i < 9; i++) {
            byte[] reply = new byte[0x170];
            res = receive(reply, 64);
            if ((reply[13] & 0xFF) == 0x80 &&
                    (reply[14] & 0xFF) == 0x03) {
                System.out.println("Step 10 successful: MCU Disabled.");
                return true;
            }
        }
        System.err.println("Step 10 Failed");
        return false;
    }


    }

