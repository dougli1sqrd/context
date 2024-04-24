# Context

**Context** is a repository for me to explore how a multi-process system would work on a single core microcontroller. The name "context" comes from the crucial component to this experiment: the "context switch". This is where your CPU is running some code, and then for some reason needs to shelve what it's working on and switch to a new task. This is common when servicing interrupt handlers.

When an interrupt comes in, the CPU jumps to an address saved in the `mvec` register (in RISC-V) and begins running code setup there. The programmer will have setup the code there to save the environment before the interrupt: CPU registers, the old Program Counter (PC), which in RISC-V is obtained after the interrupt in the `mepc` register. Then the interrupt handler runs. After it runs the old running environment - or context - is loaded back out of memory restoring the old CPU state. This means execution before the interrupt continues merrily as if nothing happened.

This is not dissimlar to switching Tasks in a multi-process system. How does that actually get set up? How does one actually make a multi process system? This is what this repo is here to help do.

# In Progress
Currently this is technically working, but not well. For example, when two tasks are using the UART to print text to the host terminal at the same time there's a chance both tasks deadlock. In part this is due to having no mediation and both tasks running in Machine mode are contesting the shared resource.

## To do
- [ ] Run tasks at a lower priv level
- [ ] Shared resources should be accessed only through a basic kernel, which mediates access
- [ ] Understand the IllegalInstruction Exceptions that sometimes occur
