//! V20x USBD -> embassy-usb-driver bridge.
//!
//! Vendored from `ch32-hal/src/usbd.rs` and stripped to a concrete
//! single-instance driver: no `T: Instance` typelevel dispatch, no
//! `Peri`/`PeripheralType`, no per-chip `pin_trait!` / `foreach_interrupt!`
//! codegen. There is exactly one USBD on the V203C8T6 and exactly two
//! USB pins (PA11 = DM, PA12 = DP), so the generic machinery only paid
//! for itself in the original HAL where many chips shared the code.
//!
//! Caller responsibilities:
//! - Configure RCC so PLL=144 MHz and USBPRE=DIV3 (-> 48 MHz USB clock).
//! - Set PA11/PA12 as outputs and drive low BEFORE calling `Driver::new`
//!   (V20x RM: "before USBD is enabled, the GPIO pins should be configured
//!   as push-pull output low to avoid the host seeing a phantom device").
//!   `Driver::new` does NOT touch GPIO; we do that in `main`.
//! - Install the USB interrupt handler that forwards to `on_usbd_irq`:
//!   ```ignore
//!   #[interrupt]
//!   unsafe fn USB_LP_CAN1_RX0() { unsafe { usbd::on_usbd_irq() } }
//!   ```

use core::future::poll_fn;
use core::marker::PhantomData;
use core::sync::atomic::{AtomicBool, Ordering};
use core::task::Poll;

use ch32_metapac::usbd::regs;
use ch32_metapac::usbd::vals::{EpType, Stat};
use ch32_metapac::{EXTEND, RCC, USBD, USBRAM};
use embassy_sync::waitqueue::AtomicWaker;
use embassy_usb_driver as driver;
use embassy_usb_driver::{
    Direction, EndpointAddress, EndpointAllocError, EndpointError, EndpointInfo, EndpointType,
    Event, Unsupported,
};

const EP_COUNT: usize = 8;
const USBRAM_SIZE: usize = 512;
const USBRAM_ALIGN: usize = 2;

static BUS_WAKER: AtomicWaker = AtomicWaker::new();
static EP0_SETUP: AtomicBool = AtomicBool::new(false);
static EP_IN_WAKERS: [AtomicWaker; EP_COUNT] = [const { AtomicWaker::new() }; EP_COUNT];
static EP_OUT_WAKERS: [AtomicWaker; EP_COUNT] = [const { AtomicWaker::new() }; EP_COUNT];
static IRQ_RESET: AtomicBool = AtomicBool::new(false);
static IRQ_SUSPEND: AtomicBool = AtomicBool::new(false);
static IRQ_RESUME: AtomicBool = AtomicBool::new(false);

/// Install this as the body of `#[interrupt] fn USB_LP_CAN1_RX0()`.
/// SAFETY: must run only from the USB_LP_CAN1_RX0 vector; touches static
/// wakers and USBD registers.
pub unsafe fn on_usbd_irq() {
    let regs = USBD;
    let istr = regs.istr().read();

    if istr.susp() {
        IRQ_SUSPEND.store(true, Ordering::Relaxed);
        regs.cntr().modify(|w| {
            w.set_fsusp(true);
            w.set_lpmode(true);
        });
        let mut clear = regs::Istr(!0);
        clear.set_susp(false);
        regs.istr().write_value(clear);
        BUS_WAKER.wake();
    }

    if istr.wkup() {
        IRQ_RESUME.store(true, Ordering::Relaxed);
        regs.cntr().modify(|w| {
            w.set_fsusp(false);
            w.set_lpmode(false);
        });
        let mut clear = regs::Istr(!0);
        clear.set_wkup(false);
        regs.istr().write_value(clear);
        BUS_WAKER.wake();
    }

    if istr.reset() {
        IRQ_RESET.store(true, Ordering::Relaxed);
        let mut clear = regs::Istr(!0);
        clear.set_reset(false);
        regs.istr().write_value(clear);
        BUS_WAKER.wake();
    }

    if istr.ctr() {
        let index = istr.ep_id() as usize;
        let mut epr = regs.epr(index).read();
        if epr.ctr_rx() {
            if index == 0 && epr.setup() {
                EP0_SETUP.store(true, Ordering::Relaxed);
            }
            EP_OUT_WAKERS[index].wake();
        }
        if epr.ctr_tx() {
            EP_IN_WAKERS[index].wake();
        }
        epr.set_dtog_rx(false);
        epr.set_dtog_tx(false);
        epr.set_stat_rx(Stat::from_bits(0));
        epr.set_stat_tx(Stat::from_bits(0));
        epr.set_ctr_rx(!epr.ctr_rx());
        epr.set_ctr_tx(!epr.ctr_tx());
        regs.epr(index).write_value(epr);
    }
}

fn convert_type(t: EndpointType) -> EpType {
    match t {
        EndpointType::Bulk => EpType::BULK,
        EndpointType::Control => EpType::CONTROL,
        EndpointType::Interrupt => EpType::INTERRUPT,
        EndpointType::Isochronous => EpType::ISO,
    }
}

fn invariant(mut r: regs::Epr) -> regs::Epr {
    r.set_ctr_rx(true);
    r.set_ctr_tx(true);
    r.set_dtog_rx(false);
    r.set_dtog_tx(false);
    r.set_stat_rx(Stat::from_bits(0));
    r.set_stat_tx(Stat::from_bits(0));
    r
}

fn align_len_up(len: u16) -> u16 {
    ((len as usize).div_ceil(USBRAM_ALIGN) * USBRAM_ALIGN) as u16
}

fn calc_out_len(len: u16) -> (u16, u16) {
    match len {
        2..=60 => (align_len_up(len), (align_len_up(len) / 2) << 10),
        61..=1024 => (
            len.div_ceil(32) * 32,
            ((len.div_ceil(32) - 1) << 10) | 0x8000,
        ),
        _ => panic!("invalid OUT length {}", len),
    }
}

mod btable {
    use super::*;

    pub(super) fn write_in(index: usize, addr: u16) {
        USBRAM.mem(index * 4).write_value(addr);
    }

    pub(super) fn write_in_len(index: usize, _addr: u16, len: u16) {
        USBRAM.mem(index * 4 + 1).write_value(len);
    }

    pub(super) fn write_out(index: usize, addr: u16, max_len_bits: u16) {
        USBRAM.mem(index * 4 + 2).write_value(addr);
        USBRAM.mem(index * 4 + 3).write_value(max_len_bits);
    }

    pub(super) fn read_out_len(index: usize) -> u16 {
        USBRAM.mem(index * 4 + 3).read()
    }
}

struct EndpointBuffer {
    addr: u16,
    len: u16,
}

impl EndpointBuffer {
    fn read(&mut self, buf: &mut [u8]) {
        let n = buf.len().min(self.len as usize);
        for i in 0..n.div_ceil(USBRAM_ALIGN) {
            let val = USBRAM.mem(self.addr as usize / USBRAM_ALIGN + i).read();
            let chunk = USBRAM_ALIGN.min(n - i * USBRAM_ALIGN);
            buf[i * USBRAM_ALIGN..][..chunk].copy_from_slice(&val.to_le_bytes()[..chunk]);
        }
    }

    fn write(&mut self, buf: &[u8]) {
        let n = buf.len().min(self.len as usize);
        for i in 0..n.div_ceil(USBRAM_ALIGN) {
            let mut val = [0u8; USBRAM_ALIGN];
            let chunk = USBRAM_ALIGN.min(n - i * USBRAM_ALIGN);
            val[..chunk].copy_from_slice(&buf[i * USBRAM_ALIGN..][..chunk]);
            let v = u16::from_le_bytes(val);
            USBRAM
                .mem(self.addr as usize / USBRAM_ALIGN + i)
                .write_value(v);
        }
    }
}

#[derive(Copy, Clone)]
struct EndpointData {
    ep_type: EndpointType,
    used_in: bool,
    used_out: bool,
}

/// USB driver.
pub struct Driver<'d> {
    _phantom: PhantomData<&'d mut ()>,
    alloc: [EndpointData; EP_COUNT],
    ep_mem_free: u16,
}

impl<'d> Driver<'d> {
    /// Enable USBD clock, perform RCC reset, pull D+ via EXTEND.usbdpu, and
    /// arm the interrupt. Caller has already configured PA11/PA12 as
    /// push-pull outputs driving low.
    pub fn new() -> Self {
        // Enable USBD peripheral clock + reset (mirrors ch32-hal's
        // `RccPeripheral::enable_and_reset` for the USBD instance).
        RCC.apb1pcenr().modify(|w| w.set_usbden(true));
        RCC.apb1prstr().modify(|w| w.set_usbdrst(true));
        RCC.apb1prstr().modify(|w| w.set_usbdrst(false));

        EXTEND.ctr().modify(|w| {
            w.set_usbdpu(true);
            w.set_usbdls(false); // full speed
        });

        let regs = USBD;
        regs.cntr().write(|w| {
            w.set_pdwn(false);
            w.set_fres(true);
        });

        // tSTARTUP: USB transceiver wakeup wait. embassy-time's block_for
        // works inside a sync init because the V20x time-driver-tim2
        // isn't running yet (we initialize USB before the executor
        // starts), so this is just a busy loop on tick reads.
        embassy_time::block_for(embassy_time::Duration::from_millis(100));

        regs.btable().write(|w| w.set_btable(0));
        BUS_WAKER.wake();

        Self {
            _phantom: PhantomData,
            alloc: [EndpointData {
                ep_type: EndpointType::Bulk,
                used_in: false,
                used_out: false,
            }; EP_COUNT],
            ep_mem_free: EP_COUNT as u16 * 8,
        }
    }

    fn alloc_ep_mem(&mut self, len: u16) -> Result<u16, EndpointAllocError> {
        if !(len as usize).is_multiple_of(USBRAM_ALIGN) {
            return Err(EndpointAllocError);
        }
        let addr = self.ep_mem_free;
        if addr + len > USBRAM_SIZE as u16 {
            return Err(EndpointAllocError);
        }
        self.ep_mem_free += len;
        Ok(addr)
    }

    fn is_endpoint_available<D: Dir>(&self, index: usize, ep_type: EndpointType) -> bool {
        if index == 0 && ep_type != EndpointType::Control {
            return false;
        }
        let ep = match self.alloc.get(index) {
            Some(ep) => ep,
            None => return false,
        };
        let used = ep.used_out || ep.used_in;
        if used && ep.ep_type == EndpointType::Isochronous {
            return false;
        }
        let used_dir = match D::dir() {
            Direction::Out => ep.used_out,
            Direction::In => ep.used_in,
        };
        !used || (ep.ep_type == ep_type && !used_dir)
    }

    fn alloc_endpoint<D: Dir>(
        &mut self,
        ep_type: EndpointType,
        ep_addr: Option<EndpointAddress>,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Endpoint<'d, D>, EndpointAllocError> {
        let index = if let Some(addr) = ep_addr {
            self.is_endpoint_available::<D>(addr.index(), ep_type)
                .then_some(addr.index())
        } else {
            (0..self.alloc.len()).find(|&i| self.is_endpoint_available::<D>(i, ep_type))
        };

        let (index, ep) = match index {
            Some(i) => (i, &mut self.alloc[i]),
            None => return Err(EndpointAllocError),
        };
        ep.ep_type = ep_type;

        let buf = match D::dir() {
            Direction::Out => {
                if ep.used_out {
                    return Err(EndpointAllocError);
                }
                ep.used_out = true;
                let (len, len_bits) = calc_out_len(max_packet_size);
                let addr = self.alloc_ep_mem(len)?;
                btable::write_out(index, addr, len_bits);
                EndpointBuffer { addr, len }
            }
            Direction::In => {
                if ep.used_in {
                    return Err(EndpointAllocError);
                }
                ep.used_in = true;
                let len = align_len_up(max_packet_size);
                let addr = self.alloc_ep_mem(len)?;
                btable::write_in(index, addr);
                EndpointBuffer { addr, len }
            }
        };

        Ok(Endpoint {
            _phantom: PhantomData,
            info: EndpointInfo {
                addr: EndpointAddress::from_parts(index, D::dir()),
                ep_type,
                max_packet_size,
                interval_ms,
            },
            buf,
        })
    }
}

impl<'d> driver::Driver<'d> for Driver<'d> {
    type EndpointOut = Endpoint<'d, Out>;
    type EndpointIn = Endpoint<'d, In>;
    type ControlPipe = ControlPipe<'d>;
    type Bus = Bus<'d>;

    fn alloc_endpoint_in(
        &mut self,
        ep_type: EndpointType,
        ep_addr: Option<EndpointAddress>,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Self::EndpointIn, EndpointAllocError> {
        self.alloc_endpoint(ep_type, ep_addr, max_packet_size, interval_ms)
    }

    fn alloc_endpoint_out(
        &mut self,
        ep_type: EndpointType,
        ep_addr: Option<EndpointAddress>,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Self::EndpointOut, EndpointAllocError> {
        self.alloc_endpoint(ep_type, ep_addr, max_packet_size, interval_ms)
    }

    fn start(mut self, control_max_packet_size: u16) -> (Self::Bus, Self::ControlPipe) {
        let ep_out = self
            .alloc_endpoint::<Out>(EndpointType::Control, None, control_max_packet_size, 0)
            .unwrap();
        let ep_in = self
            .alloc_endpoint::<In>(EndpointType::Control, None, control_max_packet_size, 0)
            .unwrap();

        let regs = USBD;
        regs.cntr().write(|w| {
            w.set_pdwn(false);
            w.set_fres(false);
            w.set_resetm(true);
            w.set_suspm(true);
            w.set_wkupm(true);
            w.set_ctrm(true);
        });

        // Enable the USB_LP_CAN1_RX0 interrupt at the PFIC. Mirrors
        // qingke-rt's `Interrupt::enable` for the typelevel handler.
        unsafe {
            use ch32_metapac::Interrupt;
            qingke::pfic::enable_interrupt(Interrupt::USB_LP_CAN1_RX0 as u8);
        }

        let mut ep_types = [EpType::BULK; EP_COUNT - 1];
        for i in 1..EP_COUNT {
            ep_types[i - 1] = convert_type(self.alloc[i].ep_type);
        }

        (
            Bus {
                _phantom: PhantomData,
                ep_types,
                inited: false,
            },
            ControlPipe {
                _phantom: PhantomData,
                max_packet_size: control_max_packet_size,
                ep_out,
                ep_in,
            },
        )
    }
}

/// USB bus.
pub struct Bus<'d> {
    _phantom: PhantomData<&'d mut ()>,
    ep_types: [EpType; EP_COUNT - 1],
    inited: bool,
}

impl<'d> driver::Bus for Bus<'d> {
    async fn poll(&mut self) -> Event {
        poll_fn(move |cx| {
            BUS_WAKER.register(cx.waker());

            if !self.inited {
                self.inited = true;
                return Poll::Ready(Event::PowerDetected);
            }

            let regs = USBD;

            if IRQ_RESUME.load(Ordering::Acquire) {
                IRQ_RESUME.store(false, Ordering::Relaxed);
                return Poll::Ready(Event::Resume);
            }

            if IRQ_RESET.load(Ordering::Acquire) {
                IRQ_RESET.store(false, Ordering::Relaxed);
                regs.daddr().write(|w| {
                    w.set_ef(true);
                    w.set_add(0);
                });
                regs.epr(0).write(|w| {
                    w.set_ep_type(EpType::CONTROL);
                    w.set_stat_rx(Stat::NAK);
                    w.set_stat_tx(Stat::NAK);
                });
                for i in 1..EP_COUNT {
                    regs.epr(i).write(|w| {
                        w.set_ea(i as _);
                        w.set_ep_type(self.ep_types[i - 1]);
                    })
                }
                for w in &EP_IN_WAKERS {
                    w.wake()
                }
                for w in &EP_OUT_WAKERS {
                    w.wake()
                }
                return Poll::Ready(Event::Reset);
            }

            if IRQ_SUSPEND.load(Ordering::Acquire) {
                IRQ_SUSPEND.store(false, Ordering::Relaxed);
                return Poll::Ready(Event::Suspend);
            }

            Poll::Pending
        })
        .await
    }

    fn endpoint_set_stalled(&mut self, ep_addr: EndpointAddress, stalled: bool) {
        let reg = USBD.epr(ep_addr.index() as _);
        match ep_addr.direction() {
            Direction::In => {
                loop {
                    let r = reg.read();
                    match r.stat_tx() {
                        Stat::DISABLED => break,
                        Stat::STALL => break,
                        _ => {
                            let want_stat = if stalled { Stat::STALL } else { Stat::NAK };
                            let mut w = invariant(r);
                            w.set_stat_tx(Stat::from_bits(
                                r.stat_tx().to_bits() ^ want_stat.to_bits(),
                            ));
                            reg.write_value(w);
                        }
                    }
                }
                EP_IN_WAKERS[ep_addr.index()].wake();
            }
            Direction::Out => {
                loop {
                    let r = reg.read();
                    match r.stat_rx() {
                        Stat::DISABLED => break,
                        Stat::STALL => break,
                        _ => {
                            let want_stat = if stalled { Stat::STALL } else { Stat::VALID };
                            let mut w = invariant(r);
                            w.set_stat_rx(Stat::from_bits(
                                r.stat_rx().to_bits() ^ want_stat.to_bits(),
                            ));
                            reg.write_value(w);
                        }
                    }
                }
                EP_OUT_WAKERS[ep_addr.index()].wake();
            }
        }
    }

    fn endpoint_is_stalled(&mut self, ep_addr: EndpointAddress) -> bool {
        let epr = USBD.epr(ep_addr.index() as _).read();
        match ep_addr.direction() {
            Direction::In => epr.stat_tx() == Stat::STALL,
            Direction::Out => epr.stat_rx() == Stat::STALL,
        }
    }

    fn endpoint_set_enabled(&mut self, ep_addr: EndpointAddress, enabled: bool) {
        let reg = USBD.epr(ep_addr.index() as _);
        match ep_addr.direction() {
            Direction::In => {
                loop {
                    let want_stat = if enabled { Stat::NAK } else { Stat::DISABLED };
                    let r = reg.read();
                    if r.stat_tx() == want_stat {
                        break;
                    }
                    let mut w = invariant(r);
                    w.set_stat_tx(Stat::from_bits(r.stat_tx().to_bits() ^ want_stat.to_bits()));
                    reg.write_value(w);
                }
                EP_IN_WAKERS[ep_addr.index()].wake();
            }
            Direction::Out => {
                loop {
                    let want_stat = if enabled { Stat::VALID } else { Stat::DISABLED };
                    let r = reg.read();
                    if r.stat_rx() == want_stat {
                        break;
                    }
                    let mut w = invariant(r);
                    w.set_stat_rx(Stat::from_bits(r.stat_rx().to_bits() ^ want_stat.to_bits()));
                    reg.write_value(w);
                }
                EP_OUT_WAKERS[ep_addr.index()].wake();
            }
        }
    }

    async fn enable(&mut self) {}
    async fn disable(&mut self) {}

    async fn remote_wakeup(&mut self) -> Result<(), Unsupported> {
        Err(Unsupported)
    }
}

trait Dir {
    fn dir() -> Direction;
}

pub enum In {}
impl Dir for In {
    fn dir() -> Direction {
        Direction::In
    }
}

pub enum Out {}
impl Dir for Out {
    fn dir() -> Direction {
        Direction::Out
    }
}

/// USB endpoint.
pub struct Endpoint<'d, D> {
    _phantom: PhantomData<(&'d mut (), D)>,
    info: EndpointInfo,
    buf: EndpointBuffer,
}

impl<'d, D> Endpoint<'d, D> {
    fn write_data(&mut self, buf: &[u8]) {
        let index = self.info.addr.index();
        self.buf.write(buf);
        btable::write_in_len(index, self.buf.addr, buf.len() as _);
    }

    fn read_data(&mut self, buf: &mut [u8]) -> Result<usize, EndpointError> {
        let index = self.info.addr.index();
        let rx_len = btable::read_out_len(index) as usize & 0x3FF;
        if rx_len > buf.len() {
            return Err(EndpointError::BufferOverflow);
        }
        self.buf.read(&mut buf[..rx_len]);
        Ok(rx_len)
    }
}

impl<'d> driver::Endpoint for Endpoint<'d, In> {
    fn info(&self) -> &EndpointInfo {
        &self.info
    }

    async fn wait_enabled(&mut self) {
        let index = self.info.addr.index();
        poll_fn(|cx| {
            EP_IN_WAKERS[index].register(cx.waker());
            if USBD.epr(index).read().stat_tx() == Stat::DISABLED {
                Poll::Pending
            } else {
                Poll::Ready(())
            }
        })
        .await;
    }
}

impl<'d> driver::Endpoint for Endpoint<'d, Out> {
    fn info(&self) -> &EndpointInfo {
        &self.info
    }

    async fn wait_enabled(&mut self) {
        let index = self.info.addr.index();
        poll_fn(|cx| {
            EP_OUT_WAKERS[index].register(cx.waker());
            if USBD.epr(index).read().stat_rx() == Stat::DISABLED {
                Poll::Pending
            } else {
                Poll::Ready(())
            }
        })
        .await;
    }
}

impl<'d> driver::EndpointOut for Endpoint<'d, Out> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, EndpointError> {
        let index = self.info.addr.index();
        let stat = poll_fn(|cx| {
            EP_OUT_WAKERS[index].register(cx.waker());
            let stat = USBD.epr(index).read().stat_rx();
            if matches!(stat, Stat::NAK | Stat::DISABLED) {
                Poll::Ready(stat)
            } else {
                Poll::Pending
            }
        })
        .await;

        if stat == Stat::DISABLED {
            return Err(EndpointError::Disabled);
        }

        let rx_len = self.read_data(buf)?;

        USBD.epr(index).write(|w| {
            w.set_ep_type(convert_type(self.info.ep_type));
            w.set_ea(self.info.addr.index() as _);
            w.set_stat_rx(Stat::from_bits(Stat::NAK.to_bits() ^ Stat::VALID.to_bits()));
            w.set_stat_tx(Stat::from_bits(0));
            w.set_ctr_rx(true);
            w.set_ctr_tx(true);
        });

        Ok(rx_len)
    }
}

impl<'d> driver::EndpointIn for Endpoint<'d, In> {
    async fn write(&mut self, buf: &[u8]) -> Result<(), EndpointError> {
        if buf.len() > self.info.max_packet_size as usize {
            return Err(EndpointError::BufferOverflow);
        }
        let index = self.info.addr.index();
        let stat = poll_fn(|cx| {
            EP_IN_WAKERS[index].register(cx.waker());
            let stat = USBD.epr(index).read().stat_tx();
            if matches!(stat, Stat::NAK | Stat::DISABLED) {
                Poll::Ready(stat)
            } else {
                Poll::Pending
            }
        })
        .await;
        if stat == Stat::DISABLED {
            return Err(EndpointError::Disabled);
        }
        self.write_data(buf);

        USBD.epr(index).write(|w| {
            w.set_ep_type(convert_type(self.info.ep_type));
            w.set_ea(self.info.addr.index() as _);
            w.set_stat_tx(Stat::from_bits(Stat::NAK.to_bits() ^ Stat::VALID.to_bits()));
            w.set_stat_rx(Stat::from_bits(0));
            w.set_ctr_rx(true);
            w.set_ctr_tx(true);
        });
        Ok(())
    }
}

/// USB control pipe.
pub struct ControlPipe<'d> {
    _phantom: PhantomData<&'d mut ()>,
    max_packet_size: u16,
    ep_in: Endpoint<'d, In>,
    ep_out: Endpoint<'d, Out>,
}

impl<'d> driver::ControlPipe for ControlPipe<'d> {
    fn max_packet_size(&self) -> usize {
        usize::from(self.max_packet_size)
    }

    async fn setup(&mut self) -> [u8; 8] {
        loop {
            poll_fn(|cx| {
                EP_OUT_WAKERS[0].register(cx.waker());
                if EP0_SETUP.load(Ordering::Relaxed) {
                    Poll::Ready(())
                } else {
                    Poll::Pending
                }
            })
            .await;
            let mut buf = [0; 8];
            let rx_len = self.ep_out.read_data(&mut buf);
            if rx_len != Ok(8) {
                continue;
            }
            EP0_SETUP.store(false, Ordering::Relaxed);
            return buf;
        }
    }

    async fn data_out(
        &mut self,
        buf: &mut [u8],
        first: bool,
        last: bool,
    ) -> Result<usize, EndpointError> {
        if first || last {
            let mut stat_rx = 0;
            let mut stat_tx = 0;
            if first {
                stat_rx ^= Stat::NAK.to_bits() ^ Stat::VALID.to_bits();
                stat_tx ^= Stat::NAK.to_bits() ^ Stat::STALL.to_bits();
            }
            if last {
                stat_tx ^= Stat::STALL.to_bits() ^ Stat::NAK.to_bits();
            }
            USBD.epr(0).write(|w| {
                w.set_ep_type(EpType::CONTROL);
                w.set_stat_rx(Stat::from_bits(stat_rx));
                w.set_stat_tx(Stat::from_bits(stat_tx));
                w.set_ctr_rx(true);
                w.set_ctr_tx(true);
            });
        }

        poll_fn(|cx| {
            EP_OUT_WAKERS[0].register(cx.waker());
            if USBD.epr(0).read().stat_rx() == Stat::NAK {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await;

        if EP0_SETUP.load(Ordering::Relaxed) {
            return Err(EndpointError::Disabled);
        }

        let rx_len = self.ep_out.read_data(buf)?;

        USBD.epr(0).write(|w| {
            w.set_ep_type(EpType::CONTROL);
            w.set_stat_rx(Stat::from_bits(if last {
                Stat::NAK.to_bits() ^ Stat::STALL.to_bits()
            } else {
                Stat::NAK.to_bits() ^ Stat::VALID.to_bits()
            }));
            w.set_ctr_rx(true);
            w.set_ctr_tx(true);
        });

        Ok(rx_len)
    }

    async fn data_in(&mut self, data: &[u8], first: bool, last: bool) -> Result<(), EndpointError> {
        if data.len() > self.ep_in.info.max_packet_size as usize {
            return Err(EndpointError::BufferOverflow);
        }
        if first || last {
            let mut stat_rx = 0;
            if first {
                stat_rx ^= Stat::NAK.to_bits() ^ Stat::STALL.to_bits();
            }
            if last {
                stat_rx ^= Stat::STALL.to_bits() ^ Stat::VALID.to_bits();
            }
            USBD.epr(0).write(|w| {
                w.set_ep_type(EpType::CONTROL);
                w.set_stat_rx(Stat::from_bits(stat_rx));
                w.set_ep_kind(last);
                w.set_ctr_rx(true);
                w.set_ctr_tx(true);
            });
        }

        poll_fn(|cx| {
            EP_IN_WAKERS[0].register(cx.waker());
            EP_OUT_WAKERS[0].register(cx.waker());
            if USBD.epr(0).read().stat_tx() == Stat::NAK {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await;

        if EP0_SETUP.load(Ordering::Relaxed) {
            return Err(EndpointError::Disabled);
        }

        self.ep_in.write_data(data);

        USBD.epr(0).write(|w| {
            w.set_ep_type(EpType::CONTROL);
            w.set_stat_tx(Stat::from_bits(Stat::NAK.to_bits() ^ Stat::VALID.to_bits()));
            w.set_ep_kind(last);
            w.set_ctr_rx(true);
            w.set_ctr_tx(true);
        });

        Ok(())
    }

    async fn accept(&mut self) {
        self.ep_in.write_data(&[]);

        let epr = USBD.epr(0).read();
        USBD.epr(0).write(|w| {
            w.set_ep_type(EpType::CONTROL);
            w.set_stat_rx(Stat::from_bits(
                epr.stat_rx().to_bits() ^ Stat::STALL.to_bits(),
            ));
            w.set_stat_tx(Stat::from_bits(
                epr.stat_tx().to_bits() ^ Stat::VALID.to_bits(),
            ));
            w.set_ctr_rx(true);
            w.set_ctr_tx(true);
        });

        poll_fn(|cx| {
            EP_IN_WAKERS[0].register(cx.waker());
            if USBD.epr(0).read().stat_tx() == Stat::NAK {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await;
    }

    async fn reject(&mut self) {
        let epr = USBD.epr(0).read();
        USBD.epr(0).write(|w| {
            w.set_ep_type(EpType::CONTROL);
            w.set_stat_rx(Stat::from_bits(
                epr.stat_rx().to_bits() ^ Stat::STALL.to_bits(),
            ));
            w.set_stat_tx(Stat::from_bits(
                epr.stat_tx().to_bits() ^ Stat::STALL.to_bits(),
            ));
            w.set_ctr_rx(true);
            w.set_ctr_tx(true);
        });
    }

    async fn accept_set_address(&mut self, addr: u8) {
        self.accept().await;
        USBD.daddr().write(|w| {
            w.set_ef(true);
            w.set_add(addr);
        });
    }
}
