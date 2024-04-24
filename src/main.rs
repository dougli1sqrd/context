#![no_std]
#![no_main]

extern crate alloc;

use core::{
    borrow::BorrowMut,
};

use alloc::vec;
use context::{TaskManager, TaskStatus};
use esp_backtrace as _;
// use esp_println::println;
use esp32c3_hal::{
    clock::{ClockControl, CpuClock}, gpio::{GpioPin, Unknown}, interrupt::{self, Priority}, peripherals::{self, Peripherals, TIMG0}, prelude::*, systimer::SystemTimer, timer::{Timer0, TimerGroup}, trapframe::TrapFrame, Rtc, Timer, IO
};
use esp_println::print;
use esp_println::println;

#[no_mangle]
extern "C" fn stop() -> ! {
    loop {
        riscv::asm::wfi();
    }
}

pub const HEAP_SIZE: usize = 0xc000;

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

fn init_heap() {
    unsafe {
        static mut HEAP_START: [u8; HEAP_SIZE] = [0; HEAP_SIZE];
        ALLOCATOR.init(HEAP_START.as_mut_ptr(), HEAP_SIZE);
    }
}

static mut TIMER0: Option<Timer<Timer0<TIMG0>>> = None;

static mut TASK_MANAGER: Option<TaskManager> = None;

fn kernel() -> ! {
    loop {
        riscv::interrupt::free(|| {
            if let Some(timer) = unsafe { TIMER0.borrow_mut() } {
                timer.start(100_u64.millis());
            }
            println!("Start loop again");
            if let Some(tm) = unsafe { TASK_MANAGER.borrow_mut() } {
                let id = tm.current;
                println!("Task {} is current", id);
                let task = tm.running_mut();
                match task.status {
                    // Start the task by calling run if the current task is Ready
                    TaskStatus::Ready => {
                        task.status = TaskStatus::Running;
                        // Lol we need interrupts to be on when we jump into the task func, so just turn them on
                        println!("Starting Task {}", id);
                        unsafe { riscv::interrupt::enable() };
                        task.run();
                    }
                    TaskStatus::Running => {}
                    TaskStatus::Stopped => {}
                    TaskStatus::Killed => {}
                }
            }
        });
        println!("wfi");
        riscv::asm::wfi();
    }
}

static mut NUM: usize = 0;

const TICKS_PER_US: u64 = SystemTimer::TICKS_PER_SECOND / 1_000_000;

fn task2() -> isize {
    loop {
        unsafe {
            print!("#{} ", NUM);
            NUM = NUM.wrapping_add(1);
        }
    }
    0
}

fn task1() -> isize {
    // fn busy_wait(delay_us: u64) {
    //     let deadline = TICKS_PER_US * delay_us + SystemTimer::now();
    //     while SystemTimer::now() < deadline { }
    // }
    pub const GPIO_MMIO_ADDRESS: usize = 0x6000_4000;
    pub const GPIO_OUT: usize = GPIO_MMIO_ADDRESS + 0x0004;
    let p: GpioPin<Unknown, 8> = unsafe { core::mem::transmute(()) };
    let mut q = p.into_push_pull_output();
    
    loop {
        // busy_wait(500_000);
        for _ in 0..10_000_000 {}
        let _ = q.set_high();
        for _ in 0..10_000_000 {}
        let _ = q.set_low();
    }
    0
}

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::configure(system.clock_control, CpuClock::Clock160MHz).freeze();

    // Disable the watchdog timers. For the ESP32-C3, this includes the Super WDT,
    // the RTC WDT, and the TIMG WDTs.
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);

    let timer_group1 = TimerGroup::new(peripherals.TIMG1, &clocks);
    let mut wdt1 = timer_group1.wdt;

    rtc.swd.disable();
    rtc.rwdt.disable();
    wdt1.disable();

    let mut group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut timer0 = group0.timer0;
    group0.wdt.disable();

    // Enable timer interrupt
    interrupt::enable(peripherals::Interrupt::TG0_T0_LEVEL, Priority::Priority6).unwrap();
    timer0.listen();
    unsafe {
        TIMER0.replace(timer0);
    }

    let _io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut p = _io.pins.gpio8.into_push_pull_output();
    p.set_high();

    init_heap();
    println!("Hello");
    // timer0.start(1000_u64.micros());

    unsafe { TASK_MANAGER.borrow_mut() }.replace(TaskManager::new(&[task1]));

    unsafe {
        riscv::interrupt::enable();
    }

    kernel()
}

#[interrupt]
fn TG0_T0_LEVEL(trap: &mut TrapFrame) {
    // Clear Interrupt
    riscv::interrupt::free(|| unsafe {
        if let Some(ref mut timer) = TIMER0.borrow_mut() {
            timer.clear_interrupt();

            println!("\n.");

            // MEPC is where we jumped from
            // Save the MEPC to the pc of the current Task struct
            // So if we set MEPC to the PC of the Task we want to jump to we'll start executing that task
            if let Some(tm) = TASK_MANAGER.borrow_mut() {
                let id = tm.current;
                let current = tm.running_mut();
                match current.status {
                    TaskStatus::Ready => {
                        // We won't do anything here as we want to start the Task
                        // outside the interrupt handler.
                    }
                    TaskStatus::Killed => {
                        // Nothing for now
                    }
                    TaskStatus::Running => {
                        // Then we'll switch
                        // The RT crate saves the prev PC (that gets written to MEPC during a trap) to the pc field of TrapFrame
                        current.pc = Some(trap.pc);
                        println!("Saving PC ({:x}) of Task {} to Task struct", trap.pc, id);
                        // Save the previous registrs in TrapFrame to the current Task
                        let TrapFrame {
                            ra,
                            t0,
                            t1,
                            t2,
                            t3,
                            t4,
                            t5,
                            t6,
                            a0,
                            a1,
                            a2,
                            a3,
                            a4,
                            a5,
                            a6,
                            a7,
                            s0,
                            s1,
                            s2,
                            s3,
                            s4,
                            s5,
                            s6,
                            s7,
                            s8,
                            s9,
                            s10,
                            s11,
                            gp,
                            tp,
                            sp,
                            ..
                        } = *trap;
                        let reg = context::Registers {
                            ra,
                            sp,
                            gp,
                            tp,
                            t0,
                            t1,
                            t2,
                            s0,
                            s1,
                            a0,
                            a1,
                            a2,
                            a3,
                            a4,
                            a5,
                            a6,
                            a7,
                            s2,
                            s3,
                            s4,
                            s5,
                            s6,
                            s7,
                            s8,
                            s9,
                            s10,
                            s11,
                            t3,
                            t4,
                            t5,
                            t6,
                        };
                        current.registers = Some(reg);
                        current.status = TaskStatus::Stopped;
                    }
                    TaskStatus::Stopped => {}
                }

                // Step to the next Task in the queue:
                tm.next_task();

                let id = tm.current;
                // This task is the one we're about to maybe run again
                let current = tm.running_mut();
                match current.status {
                    TaskStatus::Ready => {
                        println!("Next Task {} Ready in handler", id);
                        kernel();
                        // We won't do anything here as we want to start the Task
                        // outside the interrupt handler.
                    }
                    TaskStatus::Killed => {
                        // Nothing for now
                    }
                    TaskStatus::Running => {
                        println!("Next Task {} Running in handler", id);
                        // Doesn't really make sense to be here
                    }
                    TaskStatus::Stopped => {
                        println!("Next Task {} Stopped in handler", id);
                        current.status = TaskStatus::Running;
                        // We're stopped, so we want to start again.
                        // The registers are all saved in the task, so we want to
                        // load those values into the current TrapFrame so that when
                        // this handler `mret`s it'll jump into the current process
                        match (&current.registers, current.pc) {
                            (Some(reg), Some(pc)) => {
                                trap.a0 = reg.a0;
                                trap.a1 = reg.a1;
                                trap.a2 = reg.a2;
                                trap.a3 = reg.a3;
                                trap.a4 = reg.a4;
                                trap.a5 = reg.a5;
                                trap.a6 = reg.a6;
                                trap.a7 = reg.a7;
                                trap.gp = reg.gp;
                                trap.pc = pc;
                                println!(
                                    "Loading PC ({:x}) of Task {} to from struct to Stack",
                                    trap.pc, id
                                );
                                trap.ra = reg.ra;
                                trap.s0 = reg.s0;
                                trap.s1 = reg.s1;
                                trap.s2 = reg.s2;
                                trap.s3 = reg.s3;
                                trap.s4 = reg.s4;
                                trap.s5 = reg.s5;
                                trap.s6 = reg.s6;
                                trap.s7 = reg.s7;
                                trap.s8 = reg.s8;
                                trap.s9 = reg.s9;
                                trap.s10 = reg.s10;
                                trap.s11 = reg.s11;
                                trap.t0 = reg.t0;
                                trap.t1 = reg.t1;
                                trap.t2 = reg.t2;
                                trap.t3 = reg.t3;
                                trap.t4 = reg.t4;
                                trap.t5 = reg.t5;
                                trap.t6 = reg.t6;
                                trap.sp = reg.sp;
                            }
                            (None, _) => {
                                panic!("A stopped process had no saved Register information");
                            }
                            (_, None) => {
                                panic!("A stopped process had no saved PC information");
                            }
                        }
                    }
                }
            }
            timer.start(100_u64.millis());
        }
    });
}
