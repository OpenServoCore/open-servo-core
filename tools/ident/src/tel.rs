//! TEL side-channel capture: raw non-blocking reads of the LinkE CDC
//! device into the osc-ident deframer. CDC-ACM ignores the line rate, but
//! the tty MUST be raw - the default line discipline eats 0x0d/0x11/0x13.

use std::fs::File;
use std::io::Read;
use std::os::fd::AsRawFd;
use std::os::unix::fs::OpenOptionsExt;

use anyhow::{Context, Result};
use osc_ident::frame::{TelDeframer, TelFrame, TelStats};

pub struct TelSink {
    port: File,
    deframer: TelDeframer,
    frames: Vec<TelFrame>,
}

impl TelSink {
    pub fn open(path: &str, mask: u16) -> Result<Self> {
        let port = std::fs::OpenOptions::new()
            .read(true)
            .custom_flags(libc::O_NONBLOCK | libc::O_NOCTTY)
            .open(path)
            .with_context(|| format!("open tel port {path}"))?;
        // SAFETY: valid fd; termios is plain-old-data zeroed before tcgetattr.
        unsafe {
            let fd = port.as_raw_fd();
            let mut tio: libc::termios = std::mem::zeroed();
            if libc::tcgetattr(fd, &mut tio) == 0 {
                libc::cfmakeraw(&mut tio);
                tio.c_cc[libc::VMIN] = 0;
                tio.c_cc[libc::VTIME] = 0;
                libc::tcsetattr(fd, libc::TCSANOW, &tio);
                libc::tcflush(fd, libc::TCIFLUSH);
            }
        }
        Ok(Self {
            port,
            deframer: TelDeframer::new(mask).context("tel mask invalid")?,
            frames: Vec::new(),
        })
    }

    /// Pull whatever the CDC has buffered through the deframer; call often
    /// (every Pause slice) so the OS buffer never overflows.
    pub fn drain(&mut self) {
        let mut buf = [0u8; 65536];
        loop {
            match self.port.read(&mut buf) {
                Ok(0) => break,
                Ok(n) => self.deframer.push(&buf[..n], &mut self.frames),
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => break,
                Err(_) => break,
            }
        }
    }

    pub fn take_frames(&mut self) -> Vec<TelFrame> {
        std::mem::take(&mut self.frames)
    }

    pub fn stats(&self) -> TelStats {
        self.deframer.stats()
    }
}
