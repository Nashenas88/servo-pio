#![no_std]
#![deny(unsafe_op_in_unsafe_fn)]

use core::mem::MaybeUninit;

pub mod calibration;
pub mod pwm_cluster;
pub mod servo_cluster;
pub mod servo_state;

fn alloc_array<T, const LEN: usize>(initializer: impl Fn() -> T) -> [T; LEN] {
    let mut arr: [MaybeUninit<T>; LEN] = unsafe { MaybeUninit::uninit().assume_init() };
    for item in &mut arr {
        item.write(initializer());
    }

    // Safety: All entries initialized above.
    unsafe {
        (*(&MaybeUninit::<[MaybeUninit<T>; LEN]>::new(arr) as *const _
            as *const MaybeUninit<[T; LEN]>))
            .assume_init_read()
    }
}

fn alloc_array_by_index<T, const LEN: usize>(initializer: impl Fn(usize) -> T) -> [T; LEN] {
    let mut arr: [MaybeUninit<T>; LEN] = unsafe { MaybeUninit::uninit().assume_init() };
    for (i, item) in arr.iter_mut().enumerate() {
        item.write(initializer(i));
    }

    // Safety: All entries initialized above.
    unsafe {
        (*(&MaybeUninit::<[MaybeUninit<T>; LEN]>::new(arr) as *const _
            as *const MaybeUninit<[T; LEN]>))
            .assume_init_read()
    }
}
