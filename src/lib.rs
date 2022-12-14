//! # servo-pio
//!
//! This crate allows controlling up to 30 servos on rp2040-based boards. It
//! requires one PIO state machine and 2 dma channels for every servo cluster
//! created.

#![no_std]
#![deny(unsafe_op_in_unsafe_fn)]

use core::mem::MaybeUninit;

pub mod calibration;
pub mod pwm_cluster;
pub mod servo_cluster;
pub mod servo_state;

fn initialize_array<T, const LEN: usize>(initializer: impl Fn() -> T) -> [T; LEN] {
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

fn initialize_array_by_index<T, const LEN: usize>(initializer: impl Fn(usize) -> T) -> [T; LEN] {
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

fn initialize_array_from<T, U, const LEN: usize>(
    other: [T; LEN],
    initializer: impl Fn(T) -> U,
) -> [U; LEN] {
    let mut arr: [MaybeUninit<U>; LEN] = unsafe { MaybeUninit::uninit().assume_init() };
    for (item, mapped) in arr.iter_mut().zip(other.into_iter()) {
        item.write(initializer(mapped));
    }

    // Safety: All entries initialized above. arr and other have the same length.
    unsafe {
        (*(&MaybeUninit::<[MaybeUninit<U>; LEN]>::new(arr) as *const _
            as *const MaybeUninit<[U; LEN]>))
            .assume_init_read()
    }
}

fn initialize_arrays_from<T, U, V, const LEN: usize>(
    other: [T; LEN],
    initializer: impl Fn(T) -> (U, V),
) -> ([U; LEN], [V; LEN]) {
    let mut arr1: [MaybeUninit<U>; LEN] = unsafe { MaybeUninit::uninit().assume_init() };
    let mut arr2: [MaybeUninit<V>; LEN] = unsafe { MaybeUninit::uninit().assume_init() };
    for ((item1, item2), mapped) in arr1.iter_mut().zip(arr2.iter_mut()).zip(other.into_iter()) {
        let (v, w) = initializer(mapped);
        item1.write(v);
        item2.write(w);
    }

    // Safety: All entries initialized above. arr and other have the same length.
    unsafe {
        (
            (*(&MaybeUninit::<[MaybeUninit<U>; LEN]>::new(arr1) as *const _
                as *const MaybeUninit<[U; LEN]>))
                .assume_init_read(),
            (*(&MaybeUninit::<[MaybeUninit<V>; LEN]>::new(arr2) as *const _
                as *const MaybeUninit<[V; LEN]>))
                .assume_init_read(),
        )
    }
}
