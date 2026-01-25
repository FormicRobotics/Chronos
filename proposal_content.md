PROJECT PROPOSAL
Multi-Camera MIPI Synchronization System
for NVIDIA Jetson Orin NX Platform
Prepared For:
Dov Katz
Prepared By:
Umit Kayacik
Date:
December 11, 2025
Version:
2.0
1. Executive Summary
This proposal outlines the development of a custom PCB system designed to synchronize four MIPI cameras and aggregate their image streams into a single 2-lane MIPI CSI-2 output for delivery to an NVIDIA Jetson Orin NX platform. The system utilizes a Lattice CrossLink-NX FPGA for camera stream aggregation via virtual channels, enabling all four synchronized cameras to share a single CSI port. Hardware-triggered synchronization ensures sub-microsecond accuracy, with zero-copy transfer of synchronized frames directly into GPU memory for computer vision processing.
2. Project Overview
2.1 Objectives
Design and fabricate a custom PCB to interface 4x OV9281 MIPI cameras
Implement hardware-level synchronization via trigger signals
Aggregate 4 camera streams into single 2-lane MIPI output using Lattice CrossLink-NX FPGA
Develop kernel-level driver for zero-copy GPU memory transfer
Integrate IMU with synchronized trigger signal
Design for commercial-grade components to optimize cost for mass production
2.2 System Constraints
Available Jetson Interface: Single 2-lane MIPI CSI-2 port (other ports occupied by existing sensors)
Solution: Lattice CrossLink-NX FPGA for 4-to-1 MIPI virtual channel aggregation
Component Grade: Commercial temperature range (-10°C to +55°C) - no extreme environment requirements
2.3 Camera Specifications
Parameter
Specification
Camera Module
KLT-Z6MF-OV9281 V3.0
Sensor
OmniVision OV9281 (Global Shutter)
Resolution
1MP (1280 x 800)
Interface
MIPI CSI-2 (2-lane per camera)
Quantity
4 units
Cable Length
10-15cm (MIPI direct connection)
3. Technical Architecture
3.1 System Block Diagram
The proposed system architecture:
4x OV9281 Camera Modules → MIPI CSI-2 (2-lane each) → 
Lattice CrossLink-NX FPGA (4:1 aggregation with VC tagging) → 
Single 2-Lane MIPI CSI-2 Output → Jetson Orin NX
Hardware Trigger Generator → Simultaneous sync to all cameras + IMU
3.2 Hardware Design
PCB Design Tool: Altium Designer
3.2.1 Lattice CrossLink-NX FPGA Solution
The CrossLink-NX is selected as the mandatory aggregation solution due to single available CSI port constraint:
Device: Lattice CrossLink-NX (LIFCL-40 or LIFCL-17) - latest generation with improved performance
MIPI D-PHY: Hardened D-PHY IP supporting up to 2.5 Gbps/lane
Input Configuration: 4x CSI-2 RX interfaces (2-lane each) for camera inputs
Output Configuration: 1x CSI-2 TX interface (2-lane) to Jetson Orin NX
Virtual Channel Mapping: Camera 1→VC0, Camera 2→VC1, Camera 3→VC2, Camera 4→VC3
Sync Integration: FPGA generates precision trigger signal distributed to all cameras simultaneously
Future Expansion: FPGA can output additional sync signals for external sensors if needed in V2
3.2.2 MIPI CSI-2 Interface Design
Differential Pair Routing: 100Ω differential impedance with tight length matching (±0.1mm within pairs)
Lane Configuration: Each camera uses 2 data lanes + 1 clock lane (D0±, D1±, CLK±)
Data Rate: Up to 800 Mbps/lane per camera input
ESD Protection: Low-capacitance TVS diodes (<0.5pF) on all MIPI lines
Connectors: Commercial-grade 15-pin FPC connectors
3.2.3 Synchronization Trigger Circuit
Trigger Source: FPGA-generated precision timing signal (CrossLink-NX internal PLL)
Distribution: Low-skew buffer for simultaneous trigger to all 4 cameras and IMU
Camera Interface: OV9281 FSIN (Frame Sync Input) for external trigger mode
Skew Specification: <100ns between all trigger outputs
Frame Rate: Configurable up to 120fps
3.2.4 IMU Integration
Recommended IMU: BMI088 or ICM-42688-P (6-axis, commercial grade)
Sync Method: Same FPGA trigger pulse routed to IMU interrupt pin
Interface: SPI (up to 10 MHz)
3.2.5 Power Management
Camera Power: 1.8V/2.8V/1.2V rails (~150mW per camera)
FPGA Power: 1.0V core, 1.8V I/O, 2.5V PLL
Total Power Budget: ~2.5W for entire board
3.3 Software/Firmware Development
3.3.1 FPGA Firmware
Development Tool: Lattice Radiant Software
CSI-2 RX Controllers: 4x receivers with packet parsing and VC extraction
Frame Buffer: Internal SRAM for line buffering during aggregation
CSI-2 TX Controller: Packet reconstruction with VC tagging
Trigger Generator: Precision timing with configurable frame rate
3.3.2 Kernel Driver Architecture
V4L2 Subdevice Driver: OV9281 sensor control (exposure, gain, trigger mode)
NVCSI Integration: Interface with NVIDIA camera subsystem
VC Demultiplexing: Separate VC0-VC3 streams into /dev/video0-3
Device Tree: Custom DTS overlay for Orin NX
3.3.3 Zero-Copy GPU Memory Transfer
NvBuffer API: DMA-capable memory allocation
CUDA-EGL Interop: Direct mapping to CUDA memory without CPU copy
Buffer Queue: Triple-buffering to prevent frame drops
4. Scope of Work
Requirements Analysis: System requirements and interface specifications
Schematic Design: Complete circuit design with CrossLink-NX FPGA
PCB Layout: High-speed design with proper MIPI impedance control
Fabrication: PCB manufacturing and component procurement
Assembly: Board assembly and hardware testing
FPGA Firmware: CrossLink-NX firmware development
Kernel Driver: Linux driver and zero-copy implementation
Integration & Testing: Full system validation
Documentation: Technical documentation package
5. Deliverables
Complete schematic design files (Altium Designer)
PCB layout files with Gerber outputs
Bill of Materials (BOM)
3x Working prototype boards (assembled and tested)
FPGA firmware source code (Lattice Radiant project)
Kernel driver source code
Sample application demonstrating zero-copy GPU memory access
Technical documentation package
6. Project Timeline Options
6.1 Standard Timeline (10-12 Weeks)
Phase
Task
Duration
1
Requirements & Architecture
Week 1
2
Schematic Design
Weeks 2-3
3
PCB Layout & Review
Weeks 3-4
4
PCB Fabrication & Procurement
Weeks 5-6
5
Assembly & Hardware Testing
Week 7
6
FPGA Firmware + Kernel Driver
Weeks 7-9
7
Integration & Testing
Weeks 10-11
8
Documentation & Delivery
Week 12
6.2 Expedited Timeline (4-5 Weeks)
Accelerated delivery with dedicated priority focus:
Phase
Task
Duration
1
Requirements + Schematic (parallel)
Days 1-5
2
PCB Layout (expedited review)
Days 6-10
3
PCB Fab (express) + FPGA dev (parallel)
Days 11-18
4
Assembly + Driver dev (parallel)
Days 19-25
5
Integration, Testing & Delivery
Days 26-35
Expedited option includes: Express PCB fabrication, parallel development tracks, dedicated priority scheduling, daily progress updates.
7. Budget Estimate
7.1 Development Cost (NRE)
Item
Standard
Expedited
Hardware Design (Schematic + PCB)
$6,000
$10,800
PCB Fabrication (3 boards)
$800
$1,800
Components & Assembly (3 units)
$1,500
$2,200
FPGA Firmware Development
$3,000
$4,800
Kernel Driver & Software
$2,500
$3,800
Integration, Testing & Docs
$1,200
$1,800
TOTAL NRE
$15,000
$25,200
7.2 Mass Production Cost Estimate (Per Board, Excluding Cameras)
Production Volume
Estimated Unit Cost
Prototype (5 units)
~$400/unit
Small batch (10-50 units)
~$150-200/unit
Medium volume (100-500 units)
~$100-120/unit
High volume (1000+ units)
$70-90/unit
Note: Mass production estimates based on commercial-grade components, China-based manufacturing partners. Final pricing depends on component availability and exact BOM. Camera modules (KLT-Z6MF-OV9281) and Jetson Orin NX not included.
8. Assumptions & Clarifications
Single 2-lane MIPI CSI-2 port available on Jetson (CrossLink-NX aggregation required)
Camera-to-board distance: 10-15cm (MIPI direct, no serializer needed)
Commercial-grade components (-10°C to +55°C operating range)
Client provides camera modules and Jetson Orin NX for development
IMU selection finalized during requirements phase
No display output - GPU memory processing for computer vision
Future V2 may include additional sync outputs for external sensors
9. Payment Terms
30% upon project kickoff
30% upon schematic and PCB design approval
40% upon final delivery and acceptance
10. Contact Information
Umit Kayacik
Senior Electronics Engineer
I look forward to working with you on this project. Please don't hesitate to reach out if you have any questions or require additional information.
