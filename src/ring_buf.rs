use core::convert::Infallible;
use core::mem::MaybeUninit;
use cortex_m::interrupt::free as interrupt_free;

const BUF_SIZE: usize = 1024;
pub struct RingBuf<T> {
    start: usize,
    end: usize,
    data: [MaybeUninit<T>; BUF_SIZE],
}

impl<T: Copy,> RingBuf<T> {
    pub fn new() -> Self {
        interrupt_free(|_| Self {
            start: usize::default(),
            end: usize::default(),
            data: [MaybeUninit::uninit(); BUF_SIZE],
        })
    }

    pub fn len(&self) -> usize {
            if self.start <= self.end {
                self.end - self.start
            } else {
                BUF_SIZE - self.start + self.end
            }
    }

    pub fn get(&self, i: usize) -> nb::Result<T, Infallible> {
        if self.len() <= i {
            return Err(nb::Error::WouldBlock);
        }

        Ok(unsafe { self.data[self.start + i].assume_init() })
    }

    pub fn push_back(&mut self, item: T) -> nb::Result<(), Infallible> {
            if self.len() == BUF_SIZE {
                Err(nb::Error::WouldBlock)
            } else {
                unsafe { self.data[self.end].as_mut_ptr().write(item) };
                if self.end == BUF_SIZE {
                    self.end = 0;
                } else {
                    self.end += 1;
                }
                Ok(())
            }
    }

    pub fn pop_front(&mut self) -> nb::Result<T, Infallible> {
        if self.start >= self.end {
            Err(nb::Error::WouldBlock)
        } else {
            let d = self.data[self.start];
            if self.start == BUF_SIZE {
                self.start = 0;
            } else {
                self.start += 1;
            }
            Ok(unsafe { d.assume_init() })
        }
    }
}
