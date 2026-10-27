//! Serial port I/O for VinylCraft.
//!
//! Opening a cutter's port, discovering what is behind each COM name, and writing a byte
//! stream in pause-aware chunks. The pause/cancel gate lives here because it is only
//! meaningful while a write is in progress.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

#[derive(Default)]
pub(crate) struct SendControl {
    pub(crate) pause: AtomicBool,
    pub(crate) cancel: AtomicBool,
}

/// A serial port the OS reports, annotated with the USB identity of whatever is behind it.
///
/// A bare port name is not an identity. Windows reuses COM numbers in enumeration order, so
/// when a device is unplugged its name becomes free and the next device to appear can take
/// it. Sending a cut job to "COM3" because it used to be the cutter is how a job gets shouted
/// into an unrelated microcontroller while the UI reports success.
#[derive(Debug, Clone)]
pub(crate) struct PortChoice {
    pub(crate) name: String,
    /// USB vendor/product id, when the port is a USB device.
    pub(crate) usb: Option<(u16, u16)>,
    /// Human description, e.g. "USB-SERIAL CH340".
    pub(crate) label: String,
}

impl PortChoice {
    /// USB bridges that vinyl cutters actually ship with: WCH CH340/CH341, FTDI FT232,
    /// Prolific PL2303, Silicon Labs CP210x. An STM32 native CDC port (`0483:5740`) is a
    /// dev board, not a cutter — it must not be treated as one just because it has a COM name.
    pub(crate) fn looks_like_cutter(&self) -> bool {
        match self.usb {
            Some((vid, pid)) => matches!(
                (vid, pid),
                (0x1A86, 0x7523)   // CH340
                    | (0x1A86, 0x5523) // CH341
                    | (0x0403, 0x6001) // FT232R
                    | (0x0403, 0x6015) // FT231X
                    | (0x067B, 0x2303) // PL2303
                    | (0x10C4, 0xEA60) // CP2102
            ),
            None => false,
        }
    }

    pub(crate) fn display(&self) -> String {
        match self.usb {
            Some((vid, pid)) => format!("{} — {} [{:04X}:{:04X}]", self.name, self.label, vid, pid),
            None => format!("{} — {}", self.name, self.label),
        }
    }
}

/// Enumerate serial ports with the USB identity of each one.
///
/// `available_ports` is the only place the OS tells us *what* is behind a COM name; without
/// it a port is just a string and "is the cutter plugged in?" becomes unanswerable.
pub(crate) fn scan_ports() -> Vec<PortChoice> {
    use serialport::SerialPortType;
    match serialport::available_ports() {
        Ok(list) => list
            .into_iter()
            .map(|p| {
                let (usb, label) = match p.port_type {
                    SerialPortType::UsbPort(info) => {
                        let label = match (&info.manufacturer, &info.product) {
                            (Some(m), Some(d)) => format!("{m} {d}"),
                            (None, Some(d)) => d.clone(),
                            (Some(m), None) => m.clone(),
                            (None, None) => "USB serial".into(),
                        };
                        (Some((info.vid, info.pid)), label)
                    }
                    SerialPortType::BluetoothPort => (None, "Bluetooth serial".into()),
                    SerialPortType::PciPort => (None, "PCI serial".into()),
                    SerialPortType::Unknown => (None, "unknown".into()),
                };
                PortChoice { name: p.port_name, usb, label }
            })
            .collect(),
        Err(_) => Vec::new(),
    }
}

/// Write bytes and read back whatever the device says.
///
/// The reply matters as much as the write. A cutter that accepted a job usually answers
/// with a status or error string; a silent port means either a write-only machine or the
/// wrong device entirely. Without this, "Sent N bytes" is indistinguishable from "shouted
/// into a disconnected cable", which is exactly the failure mode that looks like a
/// working app.
pub(crate) fn write_serial(port_name: &str, baud: u32, bytes: &[u8]) -> Result<(usize, String), String> {
    write_serial_ctl(port_name, baud, bytes, None)
}

/// Send `bytes` in small chunks, holding the stream while the operator has paused the job.
///
/// The cutter buffers a few hundred bytes to a few kilobytes, so withholding the rest of the
/// stream stops the carriage once that buffer drains. That is the only pause a write-only
/// protocol offers: there is no pause command in DMPL to send. Chunking also keeps a long
/// contour from being pushed into a small buffer in one blocking write.
pub(crate) fn write_serial_ctl(port_name: &str, baud: u32, bytes: &[u8], ctl: Option<&Arc<SendControl>>) -> Result<(usize, String), String> {
    use serialport::FlowControl;
    // Many vinyl cutters are 3-wire (TX/RX/GND) and never drive CTS. With hardware flow
    // control the driver buffers the whole job and then waits for a clear-to-send that never
    // arrives — the app logs "Sent N bytes" while the machine sits still. Prefer the
    // configured handshake, but fall back to none so a 3-wire cable still cuts.
    //
    // Fall back only when the port cannot be OPENED with hardware flow control. If the open
    // succeeds and the write fails partway, retrying from the start would re-send bytes that
    // already left the cable and cut part of the job twice.
    let mut port = match open_port(port_name, baud, FlowControl::Hardware) {
        Ok(p) => p,
        Err(first) => open_port(port_name, baud, FlowControl::None).map_err(|e| format!("{first}; without handshake: {e}"))?,
    };
    let mut sent = 0usize;
    for chunk in bytes.chunks(64) {
        if let Some(c) = ctl {
            while c.pause.load(Ordering::Relaxed) {
                if c.cancel.load(Ordering::Relaxed) {
                    return Err("cancelled by operator".into());
                }
                std::thread::sleep(Duration::from_millis(25));
            }
            if c.cancel.load(Ordering::Relaxed) {
                return Err("cancelled by operator".into());
            }
        }
        port.write_all(chunk).map_err(|e| e.to_string())?;
        sent += chunk.len();
    }
    let _ = port.flush();
    // Long jobs are paced by hardware flow control; this is a courtesy pause so short
    // commands reach the wire before we start listening for an answer.
    std::thread::sleep(Duration::from_millis(250));
    port.set_timeout(Duration::from_millis(400)).map_err(|e| e.to_string())?;
    let mut reply = Vec::new();
    let mut buf = [0u8; 256];
    let deadline = std::time::Instant::now() + Duration::from_millis(600);
    while std::time::Instant::now() < deadline {
        match port.read(&mut buf) {
            Ok(0) => break,
            Ok(n) => {
                reply.extend_from_slice(&buf[..n]);
                if reply.len() >= 4096 {
                    break;
                }
            }
            // A read timeout is the normal end of a quiet machine, not an error.
            Err(_) => break,
        }
    }
    let text = String::from_utf8_lossy(&reply).to_string();
    Ok((sent, text))
}

/// Open the port with a given handshake. Separated from writing so the fallback can change
/// handshake without touching bytes that may already be on the wire.
pub(crate) fn open_port(port_name: &str, baud: u32, flow: serialport::FlowControl) -> Result<Box<dyn serialport::SerialPort>, String> {
    use serialport::{DataBits, Parity, StopBits};
    serialport::new(port_name, baud)
        .data_bits(DataBits::Eight)
        .parity(Parity::None)
        .stop_bits(StopBits::One)
        .flow_control(flow)
        .timeout(Duration::from_secs(5))
        .open()
        .map_err(|e| e.to_string())
}
