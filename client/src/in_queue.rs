//! IN transfers kept queued on one endpoint, taken oldest first: the
//! browser pipe's receive side, apart from the browser so the order and the
//! resume-after-drop contract run under native tests.

use std::collections::VecDeque;
use std::future::poll_fn;
use std::pin::Pin;

pub(crate) struct InQueue<F> {
    q: VecDeque<F>,
    depth: usize,
    issued: u64,
}

impl<F: Future + Unpin> InQueue<F> {
    pub(crate) fn new(depth: usize) -> Self {
        Self {
            q: VecDeque::with_capacity(depth),
            depth,
            issued: 0,
        }
    }

    /// Transfers `issue` has started, not results taken.
    pub(crate) fn issued(&self) -> u64 {
        self.issued
    }

    pub(crate) fn fill(&mut self, mut issue: impl FnMut() -> F) {
        while self.q.len() < self.depth {
            self.issued += 1;
            self.q.push_back(issue());
        }
    }

    /// The oldest transfer's result, the queue topped back up before it
    /// returns. A call dropped while the oldest pends leaves it at the
    /// front: dropping a transfer's future does not cancel the transfer,
    /// and its bytes would go to nobody.
    pub(crate) async fn next(&mut self, mut issue: impl FnMut() -> F) -> F::Output {
        self.fill(&mut issue);
        let out = poll_fn(|cx| Pin::new(self.q.front_mut().expect("filled")).poll(cx)).await;
        self.q.pop_front();
        self.fill(issue);
        out
    }

    /// Every transfer still queued, oldest first, leaving none.
    pub(crate) fn drain(&mut self) -> impl Iterator<Item = F> + use<F> {
        std::mem::take(&mut self.q).into_iter()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::cell::RefCell;
    use std::rc::Rc;
    use std::task::{Context, Poll, Waker};

    /// Transfer `i` settles when the test sets slot `i`.
    #[derive(Clone, Default)]
    struct Source(Rc<RefCell<Vec<Option<u32>>>>);

    struct Transfer {
        src: Source,
        i: usize,
    }

    impl Future for Transfer {
        type Output = u32;
        fn poll(self: Pin<&mut Self>, _: &mut Context<'_>) -> Poll<u32> {
            match self.src.0.borrow()[self.i] {
                Some(v) => Poll::Ready(v),
                None => Poll::Pending,
            }
        }
    }

    impl Source {
        fn issue(&self) -> impl FnMut() -> Transfer + '_ {
            || {
                let mut slots = self.0.borrow_mut();
                slots.push(None);
                Transfer {
                    src: self.clone(),
                    i: slots.len() - 1,
                }
            }
        }
        fn settle(&self, i: usize, v: u32) {
            self.0.borrow_mut()[i] = Some(v);
        }
        fn started(&self) -> usize {
            self.0.borrow().len()
        }
    }

    fn poll_once<T>(f: Pin<&mut impl Future<Output = T>>) -> Poll<T> {
        f.poll(&mut Context::from_waker(Waker::noop()))
    }

    fn take(q: &mut InQueue<Transfer>, src: &Source) -> Poll<u32> {
        let f = std::pin::pin!(q.next(src.issue()));
        poll_once(f)
    }

    #[test]
    fn fill_starts_depth_transfers_and_counts_them() {
        let src = Source::default();
        let mut q = InQueue::new(4);
        q.fill(src.issue());
        q.fill(src.issue());
        assert_eq!((src.started(), q.issued()), (4, 4));
    }

    #[test]
    fn results_come_oldest_first_whatever_settles_first() {
        let src = Source::default();
        let mut q = InQueue::new(3);
        q.fill(src.issue());
        src.settle(1, 11);
        src.settle(2, 12);
        assert_eq!(take(&mut q, &src), Poll::Pending);
        src.settle(0, 10);
        assert_eq!(take(&mut q, &src), Poll::Ready(10));
        assert_eq!(take(&mut q, &src), Poll::Ready(11));
        assert_eq!(take(&mut q, &src), Poll::Ready(12));
    }

    #[test]
    fn each_result_taken_starts_one_transfer() {
        let src = Source::default();
        let mut q = InQueue::new(3);
        q.fill(src.issue());
        src.settle(0, 10);
        assert_eq!(take(&mut q, &src), Poll::Ready(10));
        assert_eq!((src.started(), q.issued(), q.q.len()), (4, 4, 3));
    }

    #[test]
    fn a_dropped_take_leaves_the_oldest_for_the_next() {
        let src = Source::default();
        let mut q = InQueue::new(2);
        q.fill(src.issue());
        assert_eq!(take(&mut q, &src), Poll::Pending);
        assert_eq!(take(&mut q, &src), Poll::Pending);
        assert_eq!(q.issued(), 2);
        src.settle(0, 10);
        assert_eq!(take(&mut q, &src), Poll::Ready(10));
        assert_eq!(q.issued(), 3);
    }

    #[test]
    fn drain_hands_back_every_queued_transfer_oldest_first() {
        let src = Source::default();
        let mut q = InQueue::new(3);
        q.fill(src.issue());
        let left: Vec<usize> = q.drain().map(|t| t.i).collect();
        assert_eq!(left, [0, 1, 2]);
        assert_eq!(q.q.len(), 0);
        assert_eq!(q.issued(), 3);
    }
}
