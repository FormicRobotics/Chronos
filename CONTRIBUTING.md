# Contributing to Chronos

Thank you for your interest in contributing to the Chronos Multi-Camera Synchronization project!

## Project Structure

```
chronos/
├── fpga/                   # FPGA firmware (Lattice Radiant/SystemVerilog)
│   ├── rtl/               # RTL source files
│   ├── constraints/       # Timing and pin constraints
│   └── tb/                # Testbenches (simulation)
├── drivers/               # Linux kernel modules
│   ├── ov9281/           # OV9281 camera sensor driver
│   ├── chronos_csi/      # CSI demux platform driver
│   └── imu/              # BMI088 IMU driver
├── app/                   # User-space applications
│   └── src/              # Source code (C/C++/CUDA)
└── docs/                  # Documentation
```

## Development Setup

### FPGA Development

**Tools Required:**
- Lattice Radiant 3.2 or later
- CrossLink-NX device support installed
- ModelSim or similar for simulation

**Building FPGA:**
```bash
cd fpga
# Open project in Radiant GUI or use command line
radiantc chronos.rdf
```

### Linux Driver Development

**Prerequisites:**
- Linux kernel headers for target platform
- Cross-compiler for ARM64 (if building for Jetson)
- JetPack SDK (for Jetson development)

**Building Drivers:**
```bash
cd drivers
export CROSS_COMPILE=aarch64-linux-gnu-
export KDIR=/path/to/jetson/kernel
make all
```

### Application Development

**Prerequisites:**
- CUDA Toolkit 11.4+
- CMake 3.18+
- JetPack SDK (for NvBuffer API)

**Building Applications:**
```bash
cd app
mkdir build && cd build
cmake ..
make -j$(nproc)
```

## Coding Standards

### SystemVerilog (FPGA)

- Use `logic` instead of `wire`/`reg`
- Prefix input ports with `i_` or descriptive names
- Prefix output ports with `o_` or descriptive names
- Use `always_ff` for sequential logic
- Use `always_comb` for combinational logic
- Include comprehensive header comments
- Document all parameters and ports

**Example:**
```systemverilog
module example #(
    parameter int WIDTH = 8     // Data width
)(
    input  wire         clk,    // System clock
    input  wire         rst_n,  // Active-low reset
    input  wire  [WIDTH-1:0] data_in,
    output logic [WIDTH-1:0] data_out
);
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            data_out <= '0;
        else
            data_out <= data_in;
    end
endmodule
```

### C/C++ (Drivers and Applications)

- Follow Linux kernel coding style for drivers
- Use K&R brace style
- 4-space indentation (8-space for kernel)
- Document all public functions with Doxygen-style comments
- Use meaningful variable names
- Check all return values

**Example (Kernel Driver):**
```c
/**
 * my_function - Brief description
 * @param1: Description of parameter
 *
 * Longer description of what the function does.
 *
 * Return: 0 on success, negative error code on failure.
 */
static int my_function(int param1)
{
    int ret;
    
    ret = do_something(param1);
    if (ret < 0) {
        pr_err("Failed to do something: %d\n", ret);
        return ret;
    }
    
    return 0;
}
```

### Commit Messages

Use conventional commit format:

```
type(scope): subject

body

footer
```

**Types:**
- `feat`: New feature
- `fix`: Bug fix
- `docs`: Documentation
- `style`: Formatting, no code change
- `refactor`: Code restructuring
- `test`: Adding tests
- `chore`: Build, tools, etc.

**Examples:**
```
feat(fpga): add trigger delay calibration registers

Add per-output trigger delay registers to compensate for
cable length differences. Delay range is 0-255 clock cycles.

Closes #42
```

```
fix(driver): correct exposure calculation for high frame rates

The exposure limit was not accounting for reduced VTS at
frame rates above 60fps, causing overexposure.
```

## Testing

### FPGA Simulation

Testbenches are located in `fpga/tb/`. Run with:

```bash
cd fpga/tb
vsim -do run_tests.do
```

### Driver Testing

Basic functionality can be tested on target hardware:

```bash
# Load modules
sudo modprobe ov9281
sudo modprobe chronos_csi

# Check device creation
ls -la /dev/video*

# Verify I2C communication
i2cget -y 2 0x3c 0xf0  # Read FPGA version
```

### Application Testing

```bash
# Run synchronization test
./sync_test -n 1000 -r 120 -v

# Run demo application
./chronos_demo --rate 60 --duration 10
```

## Pull Request Process

1. Fork the repository
2. Create a feature branch: `git checkout -b feat/my-feature`
3. Make your changes with clear commits
4. Test your changes thoroughly
5. Update documentation if needed
6. Submit a pull request with clear description

## Questions?

Open an issue on GitHub or contact the maintainers.
