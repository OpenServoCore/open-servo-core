//! Main loop helpers for comms processing.
//!
//! These functions run in the main loop context (not ISR) and handle:
//!
//! - Parsing RX bytes into [`HostOp`]s
//! - Draining [`HostResult`]s and sending responses
//!
//! ## Thread Safety
//!
//! All queue operations use critical sections for safety across ISR/main boundaries.
//! This is required because `heapless::spsc` internal guarantees vary by target.

use heapless::spsc::{Consumer, Producer};

use open_servo_kernel_api::host_op::{HostError, HostOp, HostResult};

use crate::comms_service::CommsService;

/// Parse RX bytes into HostOps and enqueue them.
///
/// This function drains bytes from the RX queue, feeds them to the comms service
/// parser, and enqueues any produced operations to the op queue.
///
/// # Parameters
///
/// - `rx_cons`: Consumer end of RX byte queue (UART ISR → main)
/// - `op_prod`: Producer end of op queue (main → ADC ISR)
/// - `parser`: Comms service that parses bytes into operations
///
/// # Returns
///
/// - `Ok(count)`: Number of ops successfully enqueued
/// - `Err(())`: Op queue became full, some ops may have been rejected
///
/// When the op queue is full, the comms service receives a `Busy` error
/// via [`CommsService::push_result`] so it can respond appropriately.
///
/// # Queue Safety
///
/// All queue operations are wrapped in critical sections.
pub fn parse_and_enqueue<C, const RX_CAP: usize, const OP_CAP: usize>(
    rx_cons: &mut Consumer<'_, u8, RX_CAP>,
    op_prod: &mut Producer<'_, HostOp, OP_CAP>,
    parser: &mut C,
) -> Result<usize, ()>
where
    C: CommsService,
{
    let mut enqueued = 0;
    let mut had_full = false;

    // Drain RX bytes into parser.
    while let Some(byte) = cs_dequeue(rx_cons) {
        parser.ingest_rx_byte(byte);
    }

    // Enqueue all produced ops.
    while let Some(op) = parser.next_op() {
        if cs_enqueue(op_prod, op).is_ok() {
            enqueued += 1;
        } else {
            // Op queue full: reject with Busy error.
            parser.push_result(Err(HostError::Busy));
            had_full = true;
        }
    }

    if had_full {
        Err(())
    } else {
        Ok(enqueued)
    }
}

/// Drain results from the result queue and send responses.
///
/// This function dequeues results produced by the executor and feeds them
/// to the comms service for response formatting.
///
/// # Parameters
///
/// - `result_cons`: Consumer end of result queue (ADC ISR → main)
/// - `responder`: Comms service that formats results into responses
///
/// # Returns
///
/// Number of results processed.
///
/// # Queue Safety
///
/// All queue operations are wrapped in critical sections.
pub fn drain_and_respond<C, const RESULT_CAP: usize>(
    result_cons: &mut Consumer<'_, HostResult, RESULT_CAP>,
    responder: &mut C,
) -> usize
where
    C: CommsService,
{
    let mut processed = 0;

    while let Some(result) = cs_dequeue(result_cons) {
        responder.push_result(result);
        processed += 1;
    }

    processed
}

/// Dequeue from a Consumer within a critical section.
#[inline]
fn cs_dequeue<T, const N: usize>(cons: &mut Consumer<'_, T, N>) -> Option<T> {
    critical_section::with(|_| cons.dequeue())
}

/// Enqueue to a Producer within a critical section.
#[inline]
fn cs_enqueue<T, const N: usize>(prod: &mut Producer<'_, T, N>, val: T) -> Result<(), T> {
    critical_section::with(|_| prod.enqueue(val))
}
