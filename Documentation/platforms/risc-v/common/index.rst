===========================================================================
RISC-V Specific Features
===========================================================================

RISC-V CLIC Interrupt Threshold Configuration
==============================================

Overview
--------

The RISC-V Core-Level Interrupt Controller (CLIC) provides more flexible interrupt
control compared to the legacy CLINT. One key feature is the interrupt threshold
mechanism (INTTHRESH), which allows fine-grained control over which interrupts
are processed based on their priority levels.

CLIC Interrupt Threshold Basics
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The CLIC interrupt threshold works by:

1. **Threshold Register**: Each privilege mode has its own interrupt threshold register:
   - Machine mode: ``CSR_MINTTHRESH`` (0x347)
   - Supervisor mode: ``CSR_SINTTHRESH`` (0x147)

2. **Interrupt Filtering**: Only interrupts with priority levels above the current
   threshold are delivered to the processor.

3. **Context Preservation**: The threshold value must be saved and restored during
   context switches to maintain proper interrupt priority handling.

Configuration in Custom Chip Layer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Kconfig Configuration
"""""""""""""""""""""

To enable CLIC interrupt threshold support in your custom chip, add the following
to your chip's Kconfig file:

.. code-block:: kconfig

   config ARCH_CHIP_MYCUSTOM_CHIP
       bool "My Custom RISC-V Chip"
       select ARCH_RV32  # or ARCH_RV64
       select ARCH_RV_HAVE_CLIC
       ---help---
         My custom RISC-V chip with CLIC support

Required Definitions
""""""""""""""""""""

In your chip-specific header file (e.g., ``arch/risc-v/include/mycustom/irq.h``):

.. code-block:: c

   /* Define maximum interrupt threshold value for your CLIC implementation */
   #define RISCV_MAX_INTTHRESH    255  /* Adjust based on your CLIC design */

   /* Optional: Define interrupt priority levels */
   #define CLIC_PRIO_CRITICAL          200
   #define CLIC_PRIO_HIGH              150
   #define CLIC_PRIO_NORMAL            100
   #define CLIC_PRIO_LOW               50

.. note::
   If ``RISCV_MAX_INTTHRESH`` is not defined by the chip, NuttX will use a default
   value of ``0xff`` (255). This default should work for most CLIC implementations
   that support 8-bit interrupt threshold values. Chips with different threshold
   widths should define their own maximum value accordingly.

Implementation Details
^^^^^^^^^^^^^^^^^^^^^^

Interrupt Save/Restore Mechanism
"""""""""""""""""""""""""""""""""

When ``ARCH_RV_HAVE_CLIC`` is enabled, NuttX automatically uses the
interrupt threshold register instead of the standard status register for
interrupt control. The CLIC implementation uses the ``SWAP_CSR()`` macro
for atomic read-modify-write operations on the threshold register:

.. code-block:: c

   /* Standard RISC-V interrupt disable (without CLIC) */
   noinstrument_function static inline_function irqstate_t up_irq_save(void)
   {
     irqstate_t flags;

     /* Read mstatus & clear machine interrupt enable (MIE) in mstatus */

     __asm__ __volatile__
       (
         "csrrc %0, " __XSTR(CSR_STATUS) ", %1\n"
         : "=r" (flags)
         : "r"(STATUS_IE)
         : "memory"
       );

     return flags;
   }

   /* CLIC interrupt threshold disable (with ARCH_RV_HAVE_CLIC) */
   noinstrument_function static inline_function irqstate_t up_irq_save(void)
   {
     /* Read current interrupt threshold and set to maximum to mask all */

     return SWAP_CSR(CSR_INTTHRESH, RISCV_MAX_INTTHRESH);
   }

Context Switch Handling
"""""""""""""""""""""""

The interrupt context register (``REG_INT_CTX``) stores different values
depending on CLIC configuration:

.. code-block:: c

   /*
    * Without CLIC (standard RISC-V): REG_INT_CTX stores CSR_MSTATUS or CSR_SSTATUS
    * With CLIC (ARCH_RV_HAVE_CLIC): REG_INT_CTX stores CSR_MINTTHRESH or CSR_SINTTHRESH
    *
    * The interrupt context is automatically determined by the CONFIG_ARCH_RV_HAVE_CLIC
    * configuration, eliminating the need for a separate threshold enable option.
    */

CLIC Driver Implementation
""""""""""""""""""""""""""

To implement the CLIC driver with interrupt threshold support, you may need to configure
CLIC properly to handle priority management and threshold levels in your chip's
driver code.

Integration Checklist
^^^^^^^^^^^^^^^^^^^^^^

When integrating CLIC interrupt threshold support:

☐ Enable ``ARCH_RV_HAVE_CLIC`` in Kconfig
☐ Define ``RISCV_MAX_INTTHRESH`` for your chip (optional - defaults to 0xff/255)
☐ Implement CLIC driver with priority management
☐ Verify interrupt threshold save/restore in context switches
☐ Test interrupt priority levels and threshold functionality
☐ Add debug support for troubleshooting

References
^^^^^^^^^^

* RISC-V CLIC Specification
* NuttX RISC-V Architecture Documentation
* ``arch/risc-v/include/irq.h`` - Core interrupt handling definitions
* ``arch/risc-v/src/common/riscv_exception_common.S`` - Assembly interrupt handling

RISC-V Zicfilp Control Flow Integrity Extension
===============================================

Overview
--------

The RISC-V Zicfilp (Control Flow Integrity for Indirect Branches) extension provides
hardware support for control flow integrity by implementing branch protection mechanisms.
This extension helps protect against Return-Oriented Programming (ROP) and Jump-Oriented
Programming (JOP) attacks by adding hardware checks for indirect branch targets.

Zicfilp Extension Basics
^^^^^^^^^^^^^^^^^^^^^^^^

The Zicfilp extension provides:

1. **Hardware CFI Support**: Hardware-based control flow integrity checks for indirect branches
2. **Branch Protection**: Protection against code-reuse attacks (ROP/JOP)
3. **Compiler Integration**: Works with compiler-generated CFI instrumentation
4. **Performance**: Hardware implementation provides better performance than software-only solutions

Configuration
^^^^^^^^^^^^^

Chip Configuration Requirement
""""""""""""""""""""""""""""""

**IMPORTANT**: The ``ARCH_RV_ISA_ZICFILP`` option must be selected by the chip
configuration and should not be manually enabled by users. This ensures that
the extension is only activated on hardware that actually supports it.

To enable Zicfilp extension support in your custom RISC-V chip:

.. code-block:: kconfig

   config ARCH_CHIP_MYCUSTOM_CHIP
       bool "My Custom RISC-V Chip with Zicfilp"
       select ARCH_RV32  # or ARCH_RV64
       select ARCH_RV_ISA_M
       select ARCH_RV_ISA_A
       select ARCH_RV_ISA_C
       select ARCH_RV_ISA_ZICFILP  # Select this only if hardware supports it
       ---help---
         My custom RISC-V chip with Zicfilp CFI support

Kconfig Configuration
"""""""""""""""""""""

The actual Kconfig definition is:

.. code-block:: kconfig

   config ARCH_RV_ISA_ZICFILP
       bool
       default n
       ---help---
         Enable support for the RISC-V Zicfilp (Control Flow Integrity for Indirect Branches)
         extension. This extension provides hardware support for control flow integrity
         by implementing branch protection mechanisms. When enabled, the compiler will
         generate code with control flow protection for indirect branches.

         NOTE: This option should not be manually enabled. It must be selected by
         the chip configuration to ensure proper hardware support.

.. warning::
   **Do not manually enable this option** through menuconfig or defconfig unless you are
   creating a custom chip configuration. The chip configuration should select this option
   based on actual hardware capabilities.

Valid Configuration Methods
"""""""""""""""""""""""""""

1. **Chip Configuration** (Recommended): Add ``select ARCH_RV_ISA_ZICFILP`` to your chip's Kconfig
2. **Custom Board** (Advanced): Only if creating a custom chip configuration that supports Zicfilp

Invalid Configuration Methods
"""""""""""""""""""""""""""""

These methods should be avoided:

1. **menuconfig**: Manual selection through the menu system
2. **defconfig**: Direct addition of ``CONFIG_ARCH_RV_ISA_ZICFILP=y``

The above methods bypass hardware capability checks and may result in CFI instructions
being generated for hardware that doesn't support them.

Compiler Support
^^^^^^^^^^^^^^^^

When ``ARCH_RV_ISA_ZICFILP`` is enabled, the build system automatically:

1. **ISA String**: Adds ``_zicfilp`` to the ``-march`` compiler flag

   .. code-block:: bash

      # Example: rv64imac becomes rv64imac_zicfilp
      -march=rv64imac_zicfilp

2. **CFI Protection**: Adds ``-fcf-protection=branch`` compiler flag for branch protection

   .. code-block:: bash

      # Compiler flags
      -fcf-protection=branch

Implementation Details
^^^^^^^^^^^^^^^^^^^^^^

Hardware Requirements
"""""""""""""""""""""

To use this feature, your RISC-V implementation must:

- Support the Zicfilp extension in hardware
- Implement the required CFI instructions and checks
- Support the necessary CSRs for CFI configuration

Toolchain Requirements
""""""""""""""""""""""

- **GCC**: Version that supports Zicfilp extension and ``-fcf-protection=branch``
- **Clang**: Version that supports Zicfilp extension
- **Binutils**: Version that can assemble Zicfilp instructions

Runtime Behavior
"""""""""""""""""

When CFI is not enabled or supported at runtime, CFI-related instructions will be
treated as NOPs (no operation):

.. note::
   **CFI Instruction Behavior**: If the Zicfilp extension is not enabled in hardware
   or runtime configuration, CFI-related instructions inserted by the compiler will
   execute as NOP instructions. This provides backward compatibility but offers no
   protection.

Integration Example
^^^^^^^^^^^^^^^^^^^

For a custom RISC-V chip with Zicfilp support:

.. code-block:: kconfig

   config ARCH_CHIP_MYCUSTOM_CHIP
       bool "My Custom RISC-V Chip with CFI"
       select ARCH_RV64
       select ARCH_RV_ISA_M
       select ARCH_RV_ISA_A
       select ARCH_RV_ISA_C
       select ARCH_RV_ISA_ZICFILP  # Select only if hardware supports Zicfilp
       ---help---
         My custom RISC-V chip with Zicfilp CFI support

.. note::
   **Hardware Verification Required**: Only select ``ARCH_RV_ISA_ZICFILP`` if your
   chip actually implements the Zicfilp extension in hardware. Selecting this option
   for chips without hardware support will generate CFI instructions that execute
   as NOPs, providing no security benefit.

Debugging and Verification
^^^^^^^^^^^^^^^^^^^^^^^^^^

To verify Zicfilp is properly enabled:

1. **Check compiler flags**: Verify ``-march`` includes ``_zicfilp`` and ``-fcf-protection=branch`` is present
2. **Disassemble code**: Look for CFI-related instructions in generated code
3. **Runtime testing**: Test with CFI violations to ensure protection is active

References
^^^^^^^^^^

* RISC-V Zicfilp Extension Specification
* GCC Control Flow Protection Documentation

RISC-V Zicfiss (Shadow Stack)
-----------------------------

The RISC-V Zicfiss extension provides hardware support for shadow stack functionality,
which is used to protect against Return-Oriented Programming (ROP) attacks and other
control flow hijacking techniques. This extension maintains a separate stack for
return addresses, providing enhanced security for function calls and returns.

.. note::
   **Shadow Stack Protection**: The Zicfiss extension implements a shadow stack
   mechanism that maintains return addresses separately from the main stack. This
   provides protection against stack-based attacks that attempt to modify return
   addresses.

Configuration
^^^^^^^^^^^^^

To enable Zicfiss support in NuttX, you need to configure both the ISA extension
and the shadow stack functionality:

.. code-block:: kconfig

   config ARCH_RV_ISA_ZICFISS
       bool
       default n
       ---help---
         Enable support for the RISC-V Zicfiss (Control Flow Integrity Shadow Stack)
         extension. This extension provides hardware support for control flow integrity
         by implementing shadow stack mechanisms to protect against return-oriented
         programming (ROP) attacks.

   config ARCH_RV_SHADOW_STACK
       bool "Enable RISC-V Shadow Stack support"
       default n
       depends on ARCH_RV_ISA_ZICFISS
       ---help---
         Enable shadow stack support for RISC-V systems that have the Zicfiss
         extension. This provides additional security by maintaining a separate
         stack for return addresses.

Compiler Support
^^^^^^^^^^^^^^^^

When Zicfiss is enabled, the toolchain will:

1. **Include Zicfiss in march**: The ``_zicfiss`` extension is added to the ``-march`` flag
2. **Generate shadow stack instructions**: The compiler generates appropriate shadow stack management code
3. **CFI Protection**: Adds ``-fcf-protection=return`` compiler flag for return address protection
4. **Runtime support**: The system provides shadow stack allocation and management

   .. code-block:: bash

      # Compiler flags when shadow stack is enabled
      -march=rv64imac_zicfiss
      -fcf-protection=return

Shadow Stack Implementation
^^^^^^^^^^^^^^^^^^^^^^^^^^^

NuttX implements shadow stack support with the following features:

**Memory Management**:

- **Allocation**: Shadow stacks are allocated for each thread (typically half the size of the main stack)
- **Alignment**: Shadow stacks are aligned to 16-byte boundaries as required by RISC-V
- **Deallocation**: Shadow stacks are automatically freed when threads terminate

**Thread Context**:

- **SSP Register**: The Shadow Stack Pointer (SSP) is saved and restored during context switches
- **TCB Integration**: Shadow stack pointers are stored in the thread control block (TCB)
- **Initialization**: Shadow stacks are initialized when threads are created

**Interrupt Handling**:

- **Context Save/Restore**: The SSP register is saved and restored during interrupt processing
- **Exception Handling**: Shadow stack state is maintained across exceptions

Hardware Requirements
^^^^^^^^^^^^^^^^^^^^^

The Zicfiss extension requires:

1. **Hardware Support**: The RISC-V core must implement the Zicfiss extension
2. **CSR Support**: Access to the Shadow Stack Pointer (SSP) control and status register
3. **Memory Protection**: Proper memory management for shadow stack regions

.. warning::
   **Hardware Dependency**: Only select ``ARCH_RV_ISA_ZICFISS`` if your RISC-V
   implementation actually supports the Zicfiss extension in hardware. Enabling
   this option on hardware without Zicfiss support may result in undefined behavior.

Privilege Levels
^^^^^^^^^^^^^^^^

Similar to Zicfilp, Zicfiss behavior depends on the privilege level:

- **Machine Mode (M-mode)**: Shadow stack support depends on the specific implementation
- **Supervisor Mode (S-mode)**: Standard Zicfiss functionality should work as specified
- **User Mode (U-mode)**: Full shadow stack protection is available

For NuttX deployments:

- **Flat/Protected Mode**: Running in M-mode, shadow stack support may be implementation-specific
- **Kernel Mode**: Running in S-mode, standard Zicfiss should work as specified

Runtime Behavior
^^^^^^^^^^^^^^^^^

When shadow stack is enabled:

1. **Function Calls**: Return addresses are pushed to both the main stack and shadow stack
2. **Function Returns**: Return addresses are verified against the shadow stack
3. **Mismatch Detection**: Hardware detects mismatches between main and shadow stack return addresses
4. **Exception Generation**: Control flow violations trigger exceptions

.. note::
   **Backward Compatibility**: If the Zicfiss extension is not enabled in hardware,
   shadow stack instructions will be treated as NOPs, providing backward compatibility
   but no security protection.

Memory Overhead
^^^^^^^^^^^^^^^

Shadow stack implementation adds memory overhead:

- **Per-thread overhead**: Each thread requires an additional shadow stack (typically 50% of main stack size)
- **Kernel overhead**: Additional memory for shadow stack management structures
- **Runtime overhead**: Minimal performance impact from shadow stack operations

Integration Example
^^^^^^^^^^^^^^^^^^^

For a custom RISC-V chip with Zicfiss support:

.. code-block:: kconfig

   config ARCH_CHIP_MYCUSTOM_SECURE_CHIP
       bool "My Custom RISC-V Chip with Shadow Stack"
       select ARCH_RV64
       select ARCH_RV_ISA_M
       select ARCH_RV_ISA_A
       select ARCH_RV_ISA_C
       select ARCH_RV_ISA_ZICFISS     # Select only if hardware supports Zicfiss
       select ARCH_RV_SHADOW_STACK    # Enable shadow stack functionality
       ---help---
         My custom RISC-V chip with Zicfiss shadow stack support

Debugging and Verification
^^^^^^^^^^^^^^^^^^^^^^^^^^

To verify Zicfiss is properly enabled:

1. **Check compiler flags**: Verify ``-march`` includes ``_zicfiss`` and ``-fcf-protection=return`` is present
2. **Inspect shadow stack allocation**: Monitor shadow stack memory usage
3. **Test protection**: Use test cases that attempt ROP attacks
4. **Verify CSR access**: Ensure SSP register is accessible and functional

Troubleshooting
^^^^^^^^^^^^^^^

Common issues and solutions:

**Shadow Stack Allocation Failures**:
- Ensure sufficient memory is available for shadow stack allocation
- Check that shadow stack size is appropriate for your application

**Performance Impact**:
- Shadow stack operations have minimal overhead
- Memory usage increases due to additional shadow stacks

**Compatibility Issues**:
- Ensure all code is compiled with Zicfiss-aware toolchain
- Verify hardware actually supports the Zicfiss extension

References
^^^^^^^^^^

* RISC-V Zicfiss Extension Specification
* RISC-V Control Flow Integrity Technical Specification
* GCC Shadow Stack Documentation
* RISC-V ISA Manual - Chapter on Code Integrity Extensions
