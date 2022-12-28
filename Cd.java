package uk.co.brennan.cdapp;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FilenameFilter;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.Socket;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.text.DateFormat;
import java.util.ArrayList;
import java.util.Date;

import uk.co.brennan.cdapp.MainActivity.appstate;

import android.content.Context;
import android.graphics.Bitmap;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbEndpoint;
import android.hardware.usb.UsbInterface;
import android.hardware.usb.UsbManager;
import android.os.Environment;
import android.util.Log;
import android.view.Gravity;
import android.widget.TextView;
import android.widget.Toast;

public class Cd {

	private static UsbManager mUsbManager;
	private static UsbDevice mUsbDevice;

	private static UsbDevice mDevice;
	private static UsbDeviceConnection mConnection;

	private static UsbInterface mControlInterface;
	private static UsbInterface mDataInterface;

	private static UsbEndpoint mControlEndpoint;
	private static UsbEndpoint mReadEndpoint;
	private static UsbEndpoint mWriteEndpoint;

	private static Object mReadBufferLock = new Object();
	private static Object mWriteBufferLock = new Object();
	protected static byte[] mReadBuffer;
	protected static byte[] mWriteBuffer;

	private static MainActivity pc;

	public enum cdstate {
		FAILED, CLOSED_EMPTY, OPEN, DISC_PRESENT, NO_DISC, UNKNOWN
	};

	private static final int LONG_TIMEOUT = 5000;
	private static final int SHORT_TIMEOUT = 500;

	public static boolean ripEnabled;
	public static int percent;

	public Cd(MainActivity context) {
		pc = context;
	}

	private static byte[] toc = new byte[1000];
	private static int[] trackstart = new int[101]; // not an integer stored as
													// msf
	public static int track;
	public static int tracks;
	private static String query;
	private static byte[] status = new byte[20];

	public static Object cdlock = new Object();

	public static void testcode() {

		UsbManager usbManager = mUsbManager;
		UsbInterface uif;

		Log.d("CDAPP", "usbManager = " + usbManager);
		updtv("DeviceList.size = " + usbManager.getDeviceList().size());
		Log.d("CDAPP", "DeviceList = " + usbManager.getDeviceList());

		for (final UsbDevice usbDevice : usbManager.getDeviceList().values()) {
			// Log.d("CDAPP", "usbDevice = "+usbDevice);
			// Log.d("CDAPP", "Class = "+usbDevice.getDeviceClass()); // allways
			// zero here

			int interfaces = usbDevice.getInterfaceCount();
			for (int n = 0; n < interfaces; n++) {
				uif = usbDevice.getInterface(n);
				Log.d("CDAPP", "Interface " + n + " = " + uif);
				Log.d("CDAPP", "Interface Class = " + uif.getInterfaceClass());
				Log.d("CDAPP",
						"Interface Subclass = " + uif.getInterfaceSubclass());
			}
		}
	}

	public static UsbDevice probe(UsbManager usbManager) {

		UsbInterface uif;
		UsbDeviceConnection cx;

		mReadBuffer = new byte[16 * 1024]; // WARNING put this in constructor
											// not here
		mWriteBuffer = new byte[16 * 1024];

		for (final UsbDevice usbDevice : usbManager.getDeviceList().values()) {

			if (usbManager.hasPermission(usbDevice)) {

				int interfaces = usbDevice.getInterfaceCount();

				for (int n = 0; n < interfaces; n++) {
					uif = usbDevice.getInterface(n);

					updtv("Found " + uif.getInterfaceClass() + "."
							+ uif.getInterfaceSubclass() + " in thread "
							+ Thread.currentThread());

					if (((uif.getInterfaceClass() == 8) && (uif
							.getInterfaceSubclass() == 5)) // Duronic CD
							|| ((uif.getInterfaceClass() == 8) && (uif
									.getInterfaceSubclass() == 2)) // Samsung
																	// DVD
							|| ((uif.getInterfaceClass() == 8) && (uif
									.getInterfaceSubclass() == 6))) { // TEAC,
																		// Startech,
																		// Generic
																		// DVD

						updtv("Matching device " + usbDevice);

						cx = usbManager.openDevice(usbDevice);

						if (cx != null) {
							cx.close();
							return usbDevice;
						}

						updtv("Couldn't open that");
					}

				}
			}
		}
		return null;
	}

	public static boolean testopen(UsbManager usbManager, UsbDevice dev) {

		mDevice = dev;
		mConnection = usbManager.openDevice(mDevice);
		if (mConnection == null) {
			updtv("openDevice failed");
			return false;
		}

		// Log.d("CDAPP", "claiming interfaces, count=" +
		// mDevice.getInterfaceCount());
		// Log.d("CDAPP", "Claiming control interface.");

		mControlInterface = mDevice.getInterface(0);

		// Log.d("CDAPP", "Control iface=" + mControlInterface);

		if (!mConnection.claimInterface(mControlInterface, true)) {
			updtv("claimInterface (control) failed");
			tidy();
			return false;
		}

		mControlEndpoint = mControlInterface.getEndpoint(0);

		// Log.d("CDAPP", "Control endpoint direction: " +
		// mControlEndpoint.getDirection());
		// Log.d("CDAPP", "Claiming data interface.");

		mDataInterface = mDevice.getInterface(0);

		// Log.d("CDAPP", "data iface=" + mDataInterface);

		if (!mConnection.claimInterface(mDataInterface, true)) {
			updtv("claimInterface (data) failed");
			tidy();
			return false;
		}

		mReadEndpoint = mDataInterface.getEndpoint(0);
		// Log.d("CDAPP", "Read endpoint direction: " +
		// mReadEndpoint.getDirection());
		mWriteEndpoint = mDataInterface.getEndpoint(1);
		// Log.d("CDAPP", "Write endpoint direction: " +
		// mWriteEndpoint.getDirection());

		Log.d("CDAPP", "USB Connection Opened OK");
		return true;

	}

	public static void tidy() {

		if (mConnection != null) {
			if (mControlInterface != null) {
				mConnection.releaseInterface(mControlInterface);
				mControlInterface = null;
			}
			if (mDataInterface != null) {
				mConnection.releaseInterface(mDataInterface);
				mDataInterface = null;
			}
			mConnection.close();
			mConnection = null;
		}

	}

	public static void close() {
		mConnection.releaseInterface(mControlInterface);
		mConnection.releaseInterface(mDataInterface);
		mConnection.close();
	}

	public static int write(byte[] src, int timeoutMillis) throws IOException {
		// TODO(mikey): Nearly identical to FtdiSerial write. Refactor.
		int offset = 0;

		while (offset < src.length) {
			final int writeLength;
			final int amtWritten;

			synchronized (mWriteBufferLock) {
				final byte[] writeBuffer;

				writeLength = Math
						.min(src.length - offset, mWriteBuffer.length);
				if (offset == 0) {
					writeBuffer = src;
				} else {
					// bulkTransfer does not support offsets, make a copy.
					System.arraycopy(src, offset, mWriteBuffer, 0, writeLength);
					writeBuffer = mWriteBuffer;
				}

				amtWritten = mConnection.bulkTransfer(mWriteEndpoint,
						writeBuffer, writeLength, timeoutMillis);
			}
			if (amtWritten <= 0) {

				Log.d("CDAPP", "Write IOEXCEPTION");
				throw new IOException("Error writing " + writeLength
						+ " bytes at offset " + offset + " length="
						+ src.length);
			}

			// Log.d("CDAPP", "Wrote amt=" + amtWritten + " attempted=" +
			// writeLength);
			offset += amtWritten;
		}
		return offset;
	}

	public static int read(byte[] dest, int timeoutMillis) throws IOException {
		final int numBytesRead;
		synchronized (mReadBufferLock) {
			int readAmt = Math.min(dest.length, mReadBuffer.length);
			numBytesRead = mConnection.bulkTransfer(mReadEndpoint, mReadBuffer,
					readAmt, timeoutMillis);

			// Log.d ("CDAPP","Read Numbytes = "+numBytesRead);
			if (numBytesRead < 0) {
				// This sucks: we get -1 on timeout, not 0 as preferred.
				// We *should* use UsbRequest, except it has a bug/api oversight
				// where there is no way to determine the number of bytes read
				// in response :\ -- http://b.android.com/28023
				Log.d("CDAPP", "Read Numbytes = " + numBytesRead);
				return 0;
			}
			System.arraycopy(mReadBuffer, 0, dest, 0, numBytesRead);
		}
		return numBytesRead;
	}

	public static boolean scsi_inquiry(final byte lun) {

		int l;

		// construct command block wrapper for inquiry command
		byte[] comandBlockWrapper = { 0x55, 0x53, 0x42, 0x43, // byte 0-3:
																// dCBWSignature
				0, 0, 0, 0, // byte 4-7: dCBWTag
				0x24, 0, 0, 0, // byte 8-11: dCBWDataTransferLength
				(byte) 0x80, // byte 12: bmCBWFlags
				lun, // byte 13: bit 7-4 Reserved(0), bCBWLUN
				0x06, // byte 14: bit 7-5 Reserved(0), bCBWCBLength
				0x12, 0, 0, 0, // byte 15-30: CBWCommandBlock
				0x24, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
		// send command to device
		try {
			write(comandBlockWrapper, SHORT_TIMEOUT);
		} catch (IOException e) {
			updtv("Command write failed in scsi_inquiry");
			return false;
		}
		// WARNING check for success

		// get response and send it to caller
		byte[] temp = new byte[100];
		try {
			l = read(temp, LONG_TIMEOUT);
		} catch (IOException e) {
			return false;
		}
		if (l <= 0)
			return false;
		Log.d("CDAPP", "Inquiry: Data " + temp[0] + " " + temp[1] + " "
				+ temp[2] + " " + temp[3] + " " + temp[4] + " " + temp[5] + " "
				+ temp[6] + " " + temp[7]);

		try {
			l = read(temp, SHORT_TIMEOUT);
		} catch (IOException e) {
			return false;
		}
		if (l <= 0)
			return false;
		// Log.d ("CDAPP","Status "+temp[0]+" "+temp[1]);

		return true; // TODO
	}

	public static boolean request_sense(final byte lun) {

		int l;

		// construct command block wrapper for inquiry command
		byte[] comandBlockWrapper = { 0x55, 0x53, 0x42, 0x43, // byte 0-3:
																// dCBWSignature
				0, 0, 0, 1, // byte 4-7: dCBWTag
				0x12, 0, 0, 0, // byte 8-11: dCBWDataTransferLength
				(byte) 0x80, // byte 12: bmCBWFlags
				lun, // byte 13: bit 7-4 Reserved(0), bCBWLUN
				0x06, // byte 14: bit 7-5 Reserved(0), bCBWCBLength
				0x03, 0, 0, 0, // byte 15-30: CBWCommandBlock
				0x12, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
		// send command to device
		try {
			write(comandBlockWrapper, SHORT_TIMEOUT);
		} catch (IOException e) {
			updtv("Command write failed in request_sense");
			return false;
		}
		// check for success

		// get response and send it to caller
		byte[] temp = new byte[100];
		try {
			l = read(temp, LONG_TIMEOUT);
		} catch (IOException e) {
			return false;
		}
		if (l <= 0)
			return false;
		Log.d("CDAPP", "Request Sense: Data " + temp[0] + " " + temp[1] + " "
				+ temp[2] + " " + temp[3] + " " + temp[4] + " " + temp[5] + " "
				+ temp[6] + " " + temp[7]);

		try {
			l = read(temp, SHORT_TIMEOUT);
		} catch (IOException e) {
			return false;
		}
		if (l <= 0)
			return false;
		// Log.d ("CDAPP","Status "+temp[0]+" "+temp[1]);
		return true; // TODO
	}

	public static boolean mode_sense(final byte lun) {

		int l;

		// construct command block wrapper for inquiry command
		byte[] comandBlockWrapper = { 0x55, 0x53, 0x42, 0x43, // byte 0-3:
																// dCBWSignature
				0, 0, 0, 1, // byte 4-7: dCBWTag
				0x12, 0, 0, 0, // byte 8-11: dCBWDataTransferLength
				(byte) 0x80, // byte 12: bmCBWFlags
				lun, // byte 13: bit 7-4 Reserved(0), bCBWLUN
				0x06, // byte 14: bit 7-5 Reserved(0), bCBWCBLength
				0x5A, 0, 0, 0, // byte 15-30: CBWCommandBlock
				0x12, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
		// send command to device
		try {
			write(comandBlockWrapper, SHORT_TIMEOUT);
		} catch (IOException e) {
			updtv("Command write failed in mode_sense");
			return false;
		}
		// check for success

		// get response and send it to caller
		byte[] temp = new byte[100];
		try {
			l = read(temp, LONG_TIMEOUT);
		} catch (IOException e) {
			return false;
		}
		if (l <= 0)
			return false;
		Log.d("CDAPP", "Mode Sense: Data " + temp[0] + " " + temp[1] + " "
				+ temp[2] + " " + temp[3] + " " + temp[4] + " " + temp[5] + " "
				+ temp[6] + " " + temp[7]);

		try {
			l = read(temp, SHORT_TIMEOUT);
		} catch (IOException e) {
			return false;
		}
		if (l <= 0)
			return false;
		// Log.d ("CDAPP","Status "+temp[0]+" "+temp[1]);
		return true; // TODO
	}

	// command not recognised - locks up

	public boolean test_cd3(final byte lun) throws IOException {
		// construct command block wrapper for inquiry command
		byte[] comandBlockWrapper = { 0x55, 0x53, 0x42, 0x43, // byte 0-3:
																// dCBWSignature
				0, 0, 0, 1, // byte 4-7: dCBWTag
				0x12, 0, 0, 0, // byte 8-11: dCBWDataTransferLength
				(byte) 0x80, // byte 12: bmCBWFlags
				lun, // byte 13: bit 7-4 Reserved(0), bCBWLUN
				0x06, // byte 14: bit 7-5 Reserved(0), bCBWCBLength
				0, 1, 0, 0, // byte 15-30: CBWCommandBlock
				0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
		// send command to device
		write(comandBlockWrapper, SHORT_TIMEOUT);
		// check for success

		// get response and send it to caller
		byte[] temp = new byte[100];
		read(temp, LONG_TIMEOUT);
		Log.d("CDAPP", "Data " + temp[0] + " " + temp[1] + " " + temp[2] + " "
				+ temp[3] + " " + temp[4] + " " + temp[5] + " " + temp[6] + " "
				+ temp[7]);

		read(temp, SHORT_TIMEOUT);
		// Log.d ("CDAPP","Status "+temp[0]+" "+temp[1]);
		return false; // TODO
	}

	// Get Event Status Notification

	public static cdstate get_esn(final byte lun) {

		int l;
		cdstate r;

		// construct command block wrapper for inquiry command
		byte[] comandBlockWrapper = { 0x55, 0x53, 0x42, 0x43, // byte 0-3:
																// dCBWSignature
				0, 0, 0, 1, // byte 4-7: dCBWTag
				8, 0, 0, 0, // byte 8-11: dCBWDataTransferLength
				(byte) 0x80, // byte 12: bmCBWFlags
				lun, // byte 13: bit 7-4 Reserved(0), bCBWLUN
				0x0A, // byte 14: bit 7-5 Reserved(0), bCBWCBLength
				0x4A, 1, 0, 0, // byte 15-30: CBWCommandBlock
				0x52, 0, 0, 0, 8, 0, 0, 0, 0, 0, 0, 0 };
		// send command to device
		try {
			write(comandBlockWrapper, SHORT_TIMEOUT);
		} catch (IOException e) {
			updtv("Command write failed in get_esn");
			return cdstate.FAILED;
		}

		byte[] temp = new byte[100];
		try {
			l = read(temp, LONG_TIMEOUT);
		} catch (IOException e) {
			return cdstate.FAILED;
		}
		if (l <= 0)
			return cdstate.FAILED;

		Log.d("CDAPP", "Data " + l + " bytes " + temp[0] + " " + temp[1] + " "
				+ temp[2] + " " + temp[3] + " " + temp[4] + " " + temp[5] + " "
				+ temp[6] + " " + temp[7]);
		/*
		 * if (temp[5] == 0) Log.d ("CDAPP","Closed Empty"); else if (temp[5] ==
		 * 1) Log.d ("CDAPP","Open"); else if (temp[5] == 2) Log.d
		 * ("CDAPP","Disc Present"); else Log.d
		 * ("CDAPP","Unknown status "+temp[5]);
		 */
		if (temp[5] == 0)
			r = cdstate.CLOSED_EMPTY;
		else if (temp[5] == 1)
			r = cdstate.OPEN;
		else if (temp[5] == 2)
			r = cdstate.DISC_PRESENT;
		else
			r = cdstate.UNKNOWN;

		try {
			l = read(temp, SHORT_TIMEOUT);
		} catch (IOException e) {
			return cdstate.FAILED;
		}
		// Log.d
		// ("CDAPP","Status "+temp[0]+" "+temp[1]+" "+temp[2]+" "+temp[3]+" "+temp[4]+" "+temp[5]+" "+temp[6]+" "+temp[7]);
		if (l <= 0)
			return cdstate.FAILED;
		else
			return r;
	}

	public static cdstate test_unit_ready(final byte lun) {
		// construct command block wrapper for inquiry command
		byte[] comandBlockWrapper = { 0x55, 0x53, 0x42, 0x43, // byte 0-3:
																// dCBWSignature
				0, 0, 0, 1, // byte 4-7: dCBWTag
				0, 0, 0, 0, // byte 8-11: dCBWDataTransferLength
				0, // byte 12: bmCBWFlags
				lun, // byte 13: bit 7-4 Reserved(0), bCBWLUN
				0x06, // byte 14: bit 7-5 Reserved(0), bCBWCBLength
				0, 0, 0, 0, // byte 15-30: CBWCommandBlock
				0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
		// send command to device
		try {
			write(comandBlockWrapper, SHORT_TIMEOUT);
		} catch (IOException e) {
			updtv("Command write failed in test_unit_ready");
			return cdstate.FAILED;
		}
		// check for success

		// get response and send it to caller
		byte[] temp = new byte[100];

		int l;
		try {
			l = read(temp, LONG_TIMEOUT);
		} catch (IOException e) {
			return cdstate.FAILED;
		}
		// Log.d
		// ("CDTEST","Test Unit Ready "+temp[0]+" "+temp[1]+" "+temp[2]+" "+temp[3]+" "+temp[4]+" "+temp[5]+" "+temp[6]+" "+temp[7]);
		// Log.d ("CDTEST","Status "+temp[0]+" "+temp[1]);
		if (l != 13)
			return cdstate.FAILED;
		if (temp[12] == 0)
			return cdstate.DISC_PRESENT;
		else
			return cdstate.NO_DISC;
	}

	public static boolean read_toc(byte[] toc) {

		int l;
		// construct command block wrapper for inquiry command
		byte[] comandBlockWrapper = { 0x55, 0x53, 0x42, 0x43, // byte 0-3:
																// dCBWSignature
				0, 0, 0, 1, // byte 4-7: dCBWTag
				0x24, 3, 0, 0, // byte 8-11: dCBWDataTransferLength
				(byte) 0x80, // byte 12: bmCBWFlags
				0, // byte 13: bit 7-4 Reserved(0), bCBWLUN
				0x0A, // byte 14: bit 7-5 Reserved(0), bCBWCBLength
				0x43, 2, 0, 0, // byte 15-30: CBWCommandBlock
				0, 0, 0, 3, 0x24, 0, 0, 0, 0, 0, 0, 0 };
		// send command to device
		try {
			write(comandBlockWrapper, SHORT_TIMEOUT);
		} catch (IOException e) {
			updtv("Command write failed in read_toc");
			return false;
		}
		// check for success

		// get response and send it to caller
		byte[] temp = new byte[1000];
		try {
			l = read(toc, LONG_TIMEOUT);
		} catch (IOException e) {
			return false;
		}
		if (l <= 0)
			return false;
		/*
		 * Log.d ("CDAPP","TOC length = "+l); Log.d
		 * ("CDAPP","Data "+toc[0]+" "+toc
		 * [1]+" "+toc[2]+" "+toc[3]+" "+toc[4]+" "
		 * +toc[5]+" "+toc[6]+" "+toc[7]); Log.d
		 * ("CDAPP","Data "+toc[8]+" "+toc[
		 * 9]+" "+toc[10]+" "+toc[11]+" "+toc[12]
		 * +" "+toc[13]+" "+toc[14]+" "+toc[15]); Log.d
		 * ("CDAPP","Data "+toc[16]+
		 * " "+toc[17]+" "+toc[18]+" "+toc[19]+" "+toc[20
		 * ]+" "+toc[21]+" "+toc[22]+" "+toc[23]);
		 */
		try {
			l = read(temp, LONG_TIMEOUT);
		} catch (IOException e) {
			Log.d("CDAPP", "IOException in toc status");
			return false;
		}
		if (l <= 0) {

			Log.d("CDAPP", "timeout in toc status - try clear and retry endpoint: "+mReadEndpoint.getAddress());
			clearEndpoint(mReadEndpoint.getAddress());
//			controltest();
			try {
				l = read(temp, SHORT_TIMEOUT);
			} catch (IOException e) {
				Log.d("CDAPP", "IOException in toc status");
				return false;
			}

			Log.d("CDAPP", "Second status length = " + l);

			if (l == 13)
				return true;
			else
				return false;

		}
		// Log.d
		// ("CDAPP","Status "+temp[0]+" "+temp[1]+" "+temp[2]+" "+temp[3]+" "+temp[4]+" "+temp[5]+" "+temp[6]+" "+temp[7]);
		return true; // TODO
	}

	public static boolean read_cd(int count, int lba, byte[] buf)
			throws IOException {
		byte[] cBW = { 0x55, 0x53, 0x42, 0x43, // byte 0-3: dCBWSignature
				0, 0, 0, 1, // byte 4-7: dCBWTag
				0, 0, 0, 0, // byte 8-11: dCBWDataTransferLength
				(byte) 0x80, // byte 12: bmCBWFlags
				0, // byte 13: bit 7-4 Reserved(0), bCBWLUN
				0x0C, // byte 14: bit 7-5 Reserved(0), bCBWCBLength
				(byte) 0xBE, 0, 0, 0, // byte 15-30: CBWCommandBlock
				0, 0, 0, 0, 0, (byte) 0xF0, 0, 0, 0, 0, 0, 0 };

		int datalength = count * 2352;

		cBW[8] = (byte) (datalength & 0xff);
		cBW[9] = (byte) ((datalength >> 8) & 0xff);
		cBW[10] = (byte) ((datalength >> 16) & 0xff);
		cBW[11] = (byte) ((datalength >> 24) & 0xff);

		cBW[20] = (byte) (lba & 0xff);
		cBW[19] = (byte) ((lba >> 8) & 0xff);
		cBW[18] = (byte) ((lba >> 16) & 0xff);
		cBW[17] = (byte) ((lba >> 24) & 0xff);

		cBW[23] = (byte) (count & 0xff);
		cBW[22] = (byte) ((count >> 8) & 0xff);
		cBW[21] = (byte) ((count >> 16) & 0xff);

		// send command to device
		write(cBW, SHORT_TIMEOUT);
		// check for success
		// Log.d ("CDAPP","Read CD B "+System.currentTimeMillis());

		// get response and send it to caller

		int l = read(buf, LONG_TIMEOUT);

		// Log.d ("CDAPP","Read CD C "+System.currentTimeMillis());

		// Log.d ("CDAPP","Read CD length = "+l);

		if (l != datalength){
			Log.d ("CDAPP","Read CD failed at "+lba+" trying to clear");
			clearEndpoint(mReadEndpoint.getAddress());
//			controltest();
			return false;
		}


		read(status, SHORT_TIMEOUT);
		return true; // TODO
	}

	public static boolean eject() {

		int l;
		// construct command block wrapper for inquiry command
		byte[] comandBlockWrapper = { 0x55, 0x53, 0x42, 0x43, // byte 0-3:
																// dCBWSignature
				0, 0, 0, 1, // byte 4-7: dCBWTag
				0, 0, 0, 0, // byte 8-11: dCBWDataTransferLength
				0, // byte 12: bmCBWFlags
				0, // byte 13: bit 7-4 Reserved(0), bCBWLUN
				0x0C, // byte 14: bit 7-5 Reserved(0), bCBWCBLength
				0x1B, 0, 0, 0, // byte 15-30: CBWCommandBlock
				2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
		// send command to device
		try {
			write(comandBlockWrapper, SHORT_TIMEOUT);
		} catch (IOException e) {
			updtv("Command write failed in eject");
			return false;
		}

		updtv("Eject command sent - trying request sense");

		// check for success

		try {
			l = read(status, 10000);

			updtv("status read done");

		} catch (IOException e) {
			return false;
		}

		if (l <= 0)
			return false;

		return true;

	}

	public static boolean stopcd() {

		int l;
		// construct command block wrapper for inquiry command
		byte[] comandBlockWrapper = { 0x55, 0x53, 0x42, 0x43, // byte 0-3:
																// dCBWSignature
				0, 0, 0, 1, // byte 4-7: dCBWTag
				0, 0, 0, 0, // byte 8-11: dCBWDataTransferLength
				0, // byte 12: bmCBWFlags
				0, // byte 13: bit 7-4 Reserved(0), bCBWLUN
				0x0C, // byte 14: bit 7-5 Reserved(0), bCBWCBLength
				0x1B, 1, 0, 0, // byte 15-30: CBWCommandBlock
				0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
		// send command to device
		try {
			write(comandBlockWrapper, SHORT_TIMEOUT);
		} catch (IOException e) {
			updtv("Command write failed in stop");
			return false;
		}

		updtv("Stop command sent - trying request sense");

		// check for success

		try {
			l = read(status, 10000);

			updtv("status read done");

		} catch (IOException e) {
			return false;
		}

		if (l <= 0)
			return false;

		return true;

	}
	
	private static boolean openusb() {

		// Get UsbManager from Android.
		mUsbManager = (UsbManager) pc.getSystemService(Context.USB_SERVICE);
		// Log.d("CDAPP", "usbManager = "+mUsbManager);

		mUsbDevice = probe(mUsbManager);

		if (mUsbDevice == null)
			return false;

		if (mUsbManager.hasPermission(mUsbDevice))

			return testopen(mUsbManager, mUsbDevice);

		else {

			updtv("No permission");
			return false;
		}
	}

	public static void controltest() {

		byte buf[] = new byte[2];

		// mConnection.controlTransfer(0, 0, 0x7700, 0, buf, 0, 100);
		mConnection.controlTransfer(0x02, 0x01, 0x0000, 0x81, buf, 0,
				Cd.SHORT_TIMEOUT);

	}

	// ep should include direction flag (0x80 for IN endpoints)
	
	public static void clearEndpoint(int ep) {

		byte buf[] = new byte[2];

		// mConnection.controlTransfer(0, 0, 0x7700, 0, buf, 0, 100);
		mConnection.controlTransfer(0x02, 0x01, 0x0000, ep, buf, 0,
				Cd.SHORT_TIMEOUT);

	}	
	
	/*
	 * This function is called when a CD is attached - it polls for disc
	 * insertion and takes action it returns only if there is a problem - so the
	 * outer loop can reconnect
	 */
	private static void cd_inner_loop() {

		cdstate s, laststate;
		boolean newtoc;

		laststate = cdstate.UNKNOWN;

		while (true) {

			if (Thread.currentThread().isInterrupted()) {
				tidy();
				return;
			}

			try {
				Thread.sleep(300);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}

			synchronized (cdlock) {

				newtoc = false;

				// s = get_esn ((byte)0);
				s = test_unit_ready((byte) 0);

				if (s != laststate) {

					laststate = s;

					if (s == cdstate.FAILED) {
						updtv("CD State failed");
						tidy();
						return;
					} else if (s == cdstate.CLOSED_EMPTY) {
						updtv("CD Empty");
						updateAppStatus(appstate.NODISC);
					} else if (s == cdstate.OPEN) {
						updtv("CD Open");
						updateAppStatus(appstate.NODISC);
					} else if (s == cdstate.DISC_PRESENT) {
						updtv("Disk Present");
					} else if (s == cdstate.NO_DISC) {
						updtv("No Disk");
						updateAppStatus(appstate.NODISC);
					} else if (s == cdstate.UNKNOWN) {
						updtv("CD in unknown state");
						updateAppStatus(appstate.NODISC);
					}

					if (s == cdstate.DISC_PRESENT) {

						updateAppStatus(appstate.SEARCHING);

						test_unit_ready((byte) 0);

						if (read_toc(toc)) {
							analyse_toc();
							newtoc = true;
						} else {
							// read_toc (toc); // try again
							updtv("Table of Contents Failed");
							tidy();
							return;
						}
					}
				}
			}
			if (newtoc)
				cddb(); // crude way to get cddb outside cdlock so ripping is
						// not locked out if cddb still running

		}
	}

	public static void poll() throws IOException, InterruptedException {

		updateAppStatus(appstate.NOUSB);

		boolean opened = false;

		while (true) {

			if (Thread.currentThread().isInterrupted()) {
				Log.d("CDAPP", "********** Thread Terminated");
				tidy();
				return;
			}

			if (openusb()) {
				opened = true;
				updtv("USB opened");

				updateAppStatus(appstate.NODISC);

				cd_inner_loop();

				if (Thread.currentThread().isInterrupted()) {
					Log.d("CDAPP", "********** Thread Terminated");
					tidy();
					return;
				}

				updtv("Closing USB");
				tidy();
				updateAppStatus(appstate.NOUSB);
			}

			else {
				if (opened)
					updtv("USB disconnected");
				opened = false;
				updateAppStatus(appstate.NOUSB);
			}

			Thread.sleep(1000);
		}
	}

	private static void analyse_toc() {

		int n, i, t;

		int entries = ((0xff & toc[1]) + ((0xff & toc[0]) << 8) - 2) / 8;

		updtv("Table of Contents " + entries);

		Log.d("CDAPP", "TOC[4]: Data " + toc[4] + " " + toc[5] + " " + toc[6]
				+ " " + toc[7] + " " + toc[8] + " " + toc[9] + " " + toc[10]
				+ " " + toc[11]);
		Log.d("CDAPP", "TOC[12]: Data " + toc[12] + " " + toc[13] + " "
				+ toc[14] + " " + toc[15] + " " + toc[16] + " " + toc[17] + " "
				+ toc[18] + " " + toc[19]);

		Log.d("CDAPP", "Entries in TOC = " + entries);

		for (n = 0, i = 8; n < entries; n++, i += 8) {
			t = (toc[i] & 0xff) << 24;
			t += (toc[i + 1] & 0xff) << 16;
			t += (toc[i + 2] & 0xff) << 8;
			t += (toc[i + 3] & 0xff);

			trackstart[n] = t;
		}
		t = trackstart[0];
		Log.d("CDAPP", "trackstart " + ((t >> 16) & 0xff) + "m "
				+ ((t >> 8) & 0xff) + "s " + (t & 0xff) + "f");
		t = trackstart[entries-1];
		Log.d("CDAPP", "trackstart " + ((t >> 16) & 0xff) + "m "
				+ ((t >> 8) & 0xff) + "s " + (t & 0xff) + "f");

		int id = discid(entries - 1, trackstart);

		Log.d("CDAPP", "DISCID = " + Integer.toHexString(id));

		query = make_query(entries - 1, id, trackstart);

		tracks = entries - 1;

	}

	/*
	 * The DISCID is a 32 bit number consisting of three parts the least
	 * significant byte is the number of tracks the next 16 bits is the length
	 * of the music in seconds the most significant byte is the bottom eight
	 * bits of the sum of the digits in the decimal track start in seconds ie if
	 * two tracks 123 seconds and 456 seconds then (1+2+3+4+5+6) & 0xFF
	 */

	/*
	 * NB there is some discrepancy between the algorithm here and on JB7 On JB7
	 * trackstart[] appears to be an array of integers of frame counts in the
	 * CDDB documenetation and on disk the times are in msf I cannot see
	 * if/where JB7 converts from msf to frames for its code to work also there
	 * is a two second offset added to each trackstart in the calculation of the
	 * most significant byte of JB7 computed DISCID
	 */

	private static int cddb_sum(int n) {
		int ret;

		/* For backward compatibility this algorithm must not change */

		ret = 0;

		while (n > 0) {
			ret = ret + (n % 10);
			n = n / 10;
		}

		return (ret);
	}

	public static int discid(int tracks, int[] trackstart) {

		int i;
		int t = 0;
		int n = 0;
		int s;

		/* For backward compatibility this algorithm must not change */

		i = 0;

		while (i < tracks) {
			// n = n + cddb_sum(trackstart[i]/75+2); // frames
			s = 60 * ((trackstart[i] >> 16) & 0xff)
					+ ((trackstart[i] >> 8) & 0xff); // msf
			n = n + cddb_sum(s);
			i++;

		}

		// t = (trackstart[tracks]/75 - trackstart[0]/75); // frames
		t = 60 * ((trackstart[tracks] >> 16) & 0xff); // msf format
		t += ((trackstart[tracks] >> 8) & 0xff);
		t -= 60 * ((trackstart[0] >> 16) & 0xff);
		t -= ((trackstart[0] >> 8) & 0xff);

		Log.d("CDAPP", "DISCID t = " + t + " n = " + n);

		return ((n % 0xff) << 24 | t << 8 | tracks);

	}

	private static int msftoi(int msf) {
		int t;

		t = ((msf >> 16) & 0xff) * 60 * 75;
		t += ((msf >> 8) & 0xff) * 75;
		t += msf & 0xff;

		return t;
	}

	public static String make_query(int tracks, int discid, int trackstart[]) {

		int f, t;

		StringBuilder s = new StringBuilder();

		s.append("cddb query " + Integer.toHexString(discid) + " " + tracks);

		// track offsets in frames

		for (int n = 0; n < tracks; n++) {
			f = ((trackstart[n] >> 16) & 0xff) * 60 * 75;
			f += ((trackstart[n] >> 8) & 0xff) * 75;
			f += trackstart[n] & 0xff;
			s.append(" " + f);
		}

		// total playing time in seconds

		t = 60 * ((trackstart[tracks] >> 16) & 0xff); // msf format
		t += ((trackstart[tracks] >> 8) & 0xff);
		t -= 60 * ((trackstart[0] >> 16) & 0xff);
		t -= ((trackstart[0] >> 8) & 0xff);

		s.append(" " + t);

		Log.d("CDAPP", "Query: " + s.toString());

		return s.toString();

	}

	private static String ss;

	static void updtv(String s) {
		ss = s;

		Log.d("CDAPP", s);

		pc.handler.post((new Runnable() {
			public void run() {
				pc.setTextView(ss);
			}
		}));

	}

	static void updateAppStatus(appstate status) {
		pc.appstatus = status;
		pc.handler.post(pc.refreshAppStatus);
	}

	private static String myline(BufferedReader r) throws IOException {
		String s;

		do {
			s = r.readLine();
		} while (s.length() == 0);
		return s;
	}

	/*
	 * The goal of this method is to create a list of album metadata and post it
	 * to the UI
	 * 
	 * It is guaranteed to terminate in a fixed time
	 * 
	 * The UI thread can ignore a late result if already ripping - would be nice
	 * if I could interrupt it though.
	 */

	public static ArrayList<AlbumMetadata> albumlist = new ArrayList<AlbumMetadata>();

	public static void cddb() {
		Socket socket;
		String[] tokens;
		AlbumMetadata album;

		updtv("CDDB Started");
		albumlist.clear(); // NB remove this and it will accumulate albums for
							// testing listview
		albumlist.add(createUnknownAlbum());
		pc.handler.post(pc.updateAlbumsFromCddb);

		try {
			socket = new Socket("freedb.freedb.org", 8880);
		} catch (UnknownHostException e) {
			return;
		} catch (IOException e) {
			return;
		}

		try {
			socket.setKeepAlive(true);
			socket.setSoTimeout(10000);

			BufferedReader r = new BufferedReader(new InputStreamReader(
					socket.getInputStream()));
			PrintWriter w = new PrintWriter(socket.getOutputStream(), true);

			updtv("<--" + myline(r));

			w.print("cddb hello martin brennan.co.uk aap 0.1\r\n");
			w.flush();
			updtv("<--" + myline(r));

			updtv("-->" + query);

			w.print(query + "\r\n");
			w.flush();
			tokens = myline(r).split(" ", 4);
			if (tokens[0].equals("200")) {

				String l;

				Log.d("CDAPP", "Found exact match");

				String readquery = "cddb read " + tokens[1] + " " + tokens[2]
						+ "\r\n";
				updtv("-->" + readquery);

				w.print(readquery);
				w.flush();

				album = new AlbumMetadata();

				do {
					l = myline(r);
					if (l.charAt(0) == '#')
						;
					else {
						tokens = l.split("=", 2);
						if (tokens[0].startsWith("DTITLE")) {
	//						updtv("Album name:= " + tokens[1]);
	//						album.setname(cleanup(tokens[1]));
							tokens = tokens[1].split(" / ", 2);
	//						updtv("Artist split tokens = " + tokens.length);
							updtv("Artist **" + tokens[0] + "**");
							updtv("Album **" + tokens[1] + "**");
							if (tokens.length == 2){
								album.setname(cleanup(tokens[1]));
								album.setartist(cleanup(tokens[0]));
							}
							else {
								album.setname(cleanup(tokens[0]));
								album.setartist("Unknown");								
							}
						}
						if (tokens[0].startsWith("TTITLE")) {
							updtv("Track name:= " + tokens[1]);
							album.addtrack(cleanup(tokens[1]));
						}
					}
				} while (l.charAt(0) != '.');

				albumlist.add(album);

				pc.handler.post(pc.updateAlbumsFromCddb);

				updateAppStatus(appstate.KNOWN);

			} else
				updateAppStatus(appstate.UNKNOWN);

		} catch (SocketException e) {
			Log.d("CDAPP", "CDDB exception " + e.getMessage());
			updateAppStatus(appstate.UNKNOWN);
		} catch (UnknownHostException e) {
			Log.d("CDAPP", "CDDB exception " + e.getMessage());
			updateAppStatus(appstate.UNKNOWN);
		} catch (IOException e) {
			Log.d("CDAPP", "CDDB exception " + e.getMessage());
			updateAppStatus(appstate.UNKNOWN);
		} finally {
			try {
				socket.close();
				Log.d("CDAPP", "CDDB done");
			} catch (IOException e) {
			}
		}

	}

	public static void ripdisc() {
		AlbumMetadata album;
		File t;
		int start, end;
		synchronized (cdlock) {
			updtv("Rip started");
			updateAppStatus(appstate.RIPPING);

			try {

				album = albumlist.get(pc.selectedAlbum);

				updtv(album.albumname + " " + album.tracks.size()
						+ " CDDB tracks " + tracks + " raw tracks");
				track = 1;
				updtv("track " + ((int) 1 + track) + " from "
						+ msftoi(trackstart[track]) + " to "
						+ msftoi(trackstart[track + 1]) + " "
						+ album.tracks.get(track));
				track = 2;
				updtv("track " + ((int) 1 + track) + " from "
						+ msftoi(trackstart[track]) + " to "
						+ msftoi(trackstart[track + 1]) + " "
						+ album.tracks.get(track));

				createAlbum(album.artist,album.albumname);

				for (track = 0; (track < tracks) && ripEnabled; track++) {

					start = msftoi(trackstart[track]);
					end = msftoi(trackstart[track + 1]);
					
					if (track == (tracks - 1)) end -= 225;	// it will fail if we read run out so snip 3s

					if (pc.space() < (end - start) * 2352)
						break;

					t = doTrack(album.artist, album.albumname, album.tracks.get(track),
							start, end);
					pc.handler.post(pc.updateSpace);
					if (t == null)
						break;
				}

			} catch (IOException e) {
				updtv("IOException in ripdisc");
			}
			
			stopcd ();
			
			updateAppStatus(appstate.RIPPED);

		}
	}

	static byte[] wavheader = { 'R', 'I', 'F', 'F', 0, 0, 0, 0, 'W', 'A', 'V',
			'E', 'f', 'm', 't', ' ', 16, 0, 0, 0, 1, 0, 2, 0, 0x44,
			(byte) 0xAC, 0, 0, (byte) 0x88, 0x58, 1, 0, 4, 0, 16, 0, 'd', 'a',
			't', 'a', 0, 0, 0, 0 };

	private static void writewavheader(int len, FileOutputStream of)
			throws IOException {
		int v;

		v = len + 36;
		wavheader[4] = (byte) (v & 0xff);
		wavheader[5] = (byte) ((v >> 8) & 0xff);
		wavheader[6] = (byte) ((v >> 16) & 0xff);
		wavheader[7] = (byte) ((v >> 24) & 0xff);
		v = len;
		wavheader[40] = (byte) (v & 0xff);
		wavheader[41] = (byte) ((v >> 8) & 0xff);
		wavheader[42] = (byte) ((v >> 16) & 0xff);
		wavheader[43] = (byte) ((v >> 24) & 0xff);

		of.write(wavheader);

	}

	public static File doTrack(String artist, String album, String track, int start2, int end)
			throws IOException {

		final int SECTORS = 4;
		int count, start, lastpercent;
		byte[] temp = new byte[SECTORS * 2352];

		updtv("doTrack " + track + " from: " + start2 + " to: " + end);
		updtv("**"+artist+"**"+album+"**");
		
		start = start2;
		lastpercent = 0;

		// File f = new File ("/mnt/sda1/music/"+album+"/"+track+".wav");

		File f = new File(getMusicPath() + "/" + artist +"/" + album + "/" + track + ".wav");
//		File f = new File(getMusicPath() + "/" + album + "/" + track + ".wav");
		
		updtv("dotrack 1");
		
		if (f.exists())
			f.delete();
		
		updtv("dotrack 2");
		
		FileOutputStream of = new FileOutputStream(f);

		updtv("dotrack 3");
		
		writewavheader(2352 * (end - start2), of);

		updtv("dotrack 4");		
		
		while ((start2 < end) && ripEnabled) {
			count = Math.min(SECTORS, end - start2);
			if (!read_cd(count, start2, temp)) {
				of.close();
				return null;
			}
			start2 += count;
			of.write(temp, 0, 2352 * count);

			percent = (100 * (start2 - start)) / (end - start);
			if (percent != lastpercent) {
				lastpercent = percent;
				pc.handler.post(pc.updateRipProgress);
			}

		}
		of.close();
		return f;

	}

	public static void createAlbum(String artist, String album) throws IOException {

		File ad = new File(getMusicPath() + "/" + artist + "/" + album);
		// File ad = new File ("/mnt/sda1/music/"+album);
		// updtv ("Album album exists = "+ad.exists());

		if (!ad.exists()) { // create a new album directory if necessary
			if (!ad.mkdirs()) {
				Log.d("CDAPP", "Cannot create " + ad.getPath());
				throw new IOException();
			}
		}

	}

	private static String cleanup(String filename) {

		filename = filename.replace('/', ' ');
		return filename;
	}

	private static AlbumMetadata createUnknownAlbum() {

		AlbumMetadata album = new AlbumMetadata();
		// album.setname("Unknown Album "+DateFormat.getDateTimeInstance().format(new
		// Date()));
		album.setname("Album " + generateUnknownAlbumNo());
		album.setartist("Unknown");

		for (int n = 0; n < tracks; n++)
			album.addtrack("Track " + n);
		return album;

	}

	private static String getMusicPath() {
		String path = Environment.getExternalStorageDirectory().getPath();
		return path + "/music";
	}

	private static int generateUnknownAlbumNo() {
		File md = new File(getMusicPath()+"/Unknown");
		String[] names = md.list();

		if (names == null) return 1;
		
		int n = names.length;
		boolean found;
		do {
			n++;
			found = false;
			for (String name : names) {
				if (name.equalsIgnoreCase("album " + n)) {
					found = true;
					break;
				}
			}
		} while (found);

		return n;
	}

}
