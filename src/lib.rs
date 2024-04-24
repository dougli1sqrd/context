#![no_std]

use alloc::{vec, vec::Vec};

extern crate alloc;

pub enum ManagerStatus {
    Running,
    Switch,
}

#[derive(Debug, PartialEq)]
pub enum TaskStatus {
    Ready,
    Running,
    Stopped,
    Killed,
}

pub struct Registers {
    pub ra: usize,
    pub sp: usize,
    pub gp: usize,
    pub tp: usize,
    pub t0: usize,
    pub t1: usize,
    pub t2: usize,
    pub s0: usize,
    pub s1: usize,
    pub a0: usize,
    pub a1: usize,
    pub a2: usize,
    pub a3: usize,
    pub a4: usize,
    pub a5: usize,
    pub a6: usize,
    pub a7: usize,
    pub s2: usize,
    pub s3: usize,
    pub s4: usize,
    pub s5: usize,
    pub s6: usize,
    pub s7: usize,
    pub s8: usize,
    pub s9: usize,
    pub s10: usize,
    pub s11: usize,
    pub t3: usize,
    pub t4: usize,
    pub t5: usize,
    pub t6: usize
}

impl Registers {
    fn save_to_stack(&self) {
        
    }
}

pub struct Task {
    task: fn() -> isize,
    pub pc: Option<usize>,
    pub registers: Option<Registers>,
    pub status: TaskStatus,
}

impl Task {
    pub fn new(task: fn() -> isize) -> Task {
        Task {
            task,
            pc: None,
            registers: None,
            status: TaskStatus::Ready,
        }
    }

    pub fn run(&self) -> isize {
        // Run the task function
        (self.task)()
    }
}

pub struct TaskManager {
    pub status: ManagerStatus,
    pub tasks: Vec<Task>,
    pub current: usize,
}

impl TaskManager {
    pub fn new(tasks: &[fn() -> isize]) -> TaskManager {
        
        TaskManager {
            status: ManagerStatus::Running,
            tasks: Vec::from_iter(tasks.iter().map(|t| Task::new(*t))),
            current: 0
        }
    }

    pub fn running(&self) -> &Task {
        &self.tasks[self.current]
    }

    pub fn running_mut(&mut self) -> &mut Task {
        &mut self.tasks[self.current]
    }

    pub fn next_task(&mut self) {
        self.current = (self.current + 1) % self.tasks.len()
    }

    pub fn update(&mut self) {
        
    }
}
