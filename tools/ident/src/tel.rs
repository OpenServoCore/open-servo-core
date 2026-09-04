//! TEL side-channel capture off the LinkE CDC. A dedicated reader thread
//! drains the tty continuously into a shared buffer: at 306 kB/s the CDC
//! ring overflows in a few ms, so draining only on the pump's Pause slices
//! drops bytes and the deframer never locks (bench: 0 frames). The thread
//! blocks on the fd (short VTIME so it can see the stop flag), the pump
//! swaps the buffer out on drain().
//!
//! The tty MUST be raw (the line discipline eats 0x0d/0x11/0x13) with
//! CLOCAL|CREAD set and the speed configured - a bare O_NONBLOCK open
//! reads almost nothing (that combination is what pyserial sets up).

use std::io::Read;
use std::os::fd::AsRawFd;
use std::os::unix::fs::OpenOptionsExt;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;

use anyhow::{Context, Result};
use osc_ident::frame::{TelDeframer, TelFrame, TelStats};

pub struct TelSink {
    rx: Arc<Mutex<Vec<u8>>>,
    stop: Arc<AtomicBool>,
    bytes: Arc<AtomicU64>,
    reader: Option<JoinHandle<()>>,
    deframer: TelDeframer,
    frames: Vec<TelFrame>,
}

impl TelSink {
    pub fn open(path: &str, mask: u16) -> Result<Self> {
        let mut port = std::fs::OpenOptions::new()
            .read(true)
            .custom_flags(libc::O_NONBLOCK | libc::O_NOCTTY)
            .open(path)
            .with_context(|| format!("open tel port {path}"))?;
        let fd = port.as_raw_fd();
        // SAFETY: valid fd; termios is plain-old-data zeroed before tcgetattr.
        unsafe {
            let mut tio: libc::termios = std::mem::zeroed();
            if libc::tcgetattr(fd, &mut tio) != 0 {
                anyhow::bail!("tcgetattr {path}");
            }
            libc::cfmakeraw(&mut tio);
            // CLOCAL (ignore modem lines) + CREAD (enable receiver) are what
            // actually start the byte flow; the speed is cosmetic for CDC but
            // the driver wants it set.
            tio.c_cflag |= libc::CLOCAL | libc::CREAD;
            libc::cfsetispeed(&mut tio, libc::B115200);
            libc::cfsetospeed(&mut tio, libc::B115200);
            // VMIN 0 + VTIME 1 (0.1 s): a blocking read returns on data or
            // after the idle tick, so the reader thread can poll the stop flag.
            tio.c_cc[libc::VMIN] = 0;
            tio.c_cc[libc::VTIME] = 1;
            if libc::tcsetattr(fd, libc::TCSANOW, &tio) != 0 {
                anyhow::bail!("tcsetattr {path}");
            }
            libc::tcflush(fd, libc::TCIFLUSH);
            // Clear O_NONBLOCK: the reader thread wants blocking reads so it
            // drains continuously instead of spinning on WouldBlock.
            libc::fcntl(fd, libc::F_SETFL, 0);
        }

        let rx = Arc::new(Mutex::new(Vec::new()));
        let stop = Arc::new(AtomicBool::new(false));
        let bytes = Arc::new(AtomicU64::new(0));
        let reader = {
            let rx = rx.clone();
            let stop = stop.clone();
            let bytes = bytes.clone();
            std::thread::spawn(move || {
                let mut buf = [0u8; 65536];
                while !stop.load(Ordering::Relaxed) {
                    match port.read(&mut buf) {
                        Ok(0) => {}
                        Ok(n) => {
                            bytes.fetch_add(n as u64, Ordering::Relaxed);
                            rx.lock().expect("tel buf").extend_from_slice(&buf[..n]);
                        }
                        Err(_) => break,
                    }
                }
            })
        };

        Ok(Self {
            rx,
            stop,
            bytes,
            reader: Some(reader),
            deframer: TelDeframer::new(mask).context("tel mask invalid")?,
            frames: Vec::new(),
        })
    }

    /// Feed everything the reader thread has buffered through the deframer.
    pub fn drain(&mut self) {
        let bytes = std::mem::take(&mut *self.rx.lock().expect("tel buf"));
        if !bytes.is_empty() {
            self.deframer.push(&bytes, &mut self.frames);
        }
    }

    pub fn take_frames(&mut self) -> Vec<TelFrame> {
        std::mem::take(&mut self.frames)
    }

    pub fn stats(&self) -> TelStats {
        self.deframer.stats()
    }

    pub fn bytes_read(&self) -> u64 {
        self.bytes.load(Ordering::Relaxed)
    }
}

impl Drop for TelSink {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(h) = self.reader.take() {
            let _ = h.join();
        }
    }
}
