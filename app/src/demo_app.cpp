/*
 * Chronos Demo Application
 *
 * Copyright (C) 2025 Chronos Project
 *
 * Demonstrates synchronized 4-camera capture with IMU overlay
 * and CUDA-accelerated image processing
 */

#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <signal.h>
#include <getopt.h>

#include <cuda_runtime.h>

#include "chronos_capture.h"

/* OpenCV for display (optional) */
#ifdef USE_OPENCV
#include <opencv2/opencv.hpp>
#include <opencv2/cudaarithm.hpp>
#endif

/* ============================================================================
 * CUDA Kernels for Image Processing
 * ============================================================================ */

/* Convert 10-bit to 8-bit with optional histogram equalization */
__global__ void convert_10bit_to_8bit_kernel(
    const uint16_t* __restrict__ input,
    uint8_t* __restrict__ output,
    int width, int height, int pitch_in, int pitch_out,
    bool equalize)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    
    if (x >= width || y >= height) return;
    
    int idx_in = y * (pitch_in / sizeof(uint16_t)) + x;
    int idx_out = y * pitch_out + x;
    
    uint16_t val = input[idx_in];
    
    /* Scale from 10-bit to 8-bit */
    output[idx_out] = (uint8_t)((val >> 2) & 0xFF);
}

/* Create quad view (2x2 grid) */
__global__ void create_quad_view_kernel(
    const uint8_t* __restrict__ cam0,
    const uint8_t* __restrict__ cam1,
    const uint8_t* __restrict__ cam2,
    const uint8_t* __restrict__ cam3,
    uint8_t* __restrict__ output,
    int src_width, int src_height, int src_pitch,
    int dst_width, int dst_height, int dst_pitch)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    
    if (x >= dst_width || y >= dst_height) return;
    
    int half_w = dst_width / 2;
    int half_h = dst_height / 2;
    
    /* Determine which camera and source coordinates */
    const uint8_t* src;
    int src_x, src_y;
    
    if (x < half_w && y < half_h) {
        src = cam0;
        src_x = x * src_width / half_w;
        src_y = y * src_height / half_h;
    } else if (x >= half_w && y < half_h) {
        src = cam1;
        src_x = (x - half_w) * src_width / half_w;
        src_y = y * src_height / half_h;
    } else if (x < half_w && y >= half_h) {
        src = cam2;
        src_x = x * src_width / half_w;
        src_y = (y - half_h) * src_height / half_h;
    } else {
        src = cam3;
        src_x = (x - half_w) * src_width / half_w;
        src_y = (y - half_h) * src_height / half_h;
    }
    
    output[y * dst_pitch + x] = src[src_y * src_pitch + src_x];
}

/* ============================================================================
 * Application Class
 * ============================================================================ */

class ChronosDemo {
public:
    struct Options {
        int frame_rate = 30;
        int exposure_us = 5000;
        float gain_db = 0.0f;
        bool show_imu = true;
        bool headless = false;
        int duration_sec = 0;  /* 0 = run until stopped */
    };
    
    ChronosDemo(const Options& opts) : opts_(opts), running_(false) {
        /* Allocate GPU buffers */
        size_t frame_size = CHRONOS_FRAME_WIDTH * CHRONOS_FRAME_HEIGHT;
        
        for (int i = 0; i < CHRONOS_NUM_CAMERAS; i++) {
            cudaMalloc(&gpu_frames_8bit_[i], frame_size);
        }
        
        /* Quad view buffer (1280x800 display) */
        cudaMalloc(&gpu_quad_view_, CHRONOS_FRAME_WIDTH * CHRONOS_FRAME_HEIGHT);
        
        /* Host buffer for display */
        host_display_ = new uint8_t[CHRONOS_FRAME_WIDTH * CHRONOS_FRAME_HEIGHT];
    }
    
    ~ChronosDemo() {
        for (int i = 0; i < CHRONOS_NUM_CAMERAS; i++) {
            if (gpu_frames_8bit_[i]) cudaFree(gpu_frames_8bit_[i]);
        }
        if (gpu_quad_view_) cudaFree(gpu_quad_view_);
        if (host_display_) delete[] host_display_;
    }
    
    int run() {
        /* Initialize capture system */
        chronos_error_t err = chronos_init();
        if (err != CHRONOS_OK) {
            std::cerr << "Failed to initialize: " << chronos_strerror(err) << std::endl;
            return 1;
        }
        
        /* Configure */
        chronos_config_t config = {};
        config.frame_rate = opts_.frame_rate;
        config.exposure_us = opts_.exposure_us;
        config.gain_db = opts_.gain_db;
        config.external_trigger = true;
        config.enable_imu = opts_.show_imu;
        config.buffer_count = CHRONOS_BUFFER_COUNT;
        
        err = chronos_configure(&config);
        if (err != CHRONOS_OK) {
            std::cerr << "Failed to configure: " << chronos_strerror(err) << std::endl;
            chronos_shutdown();
            return 1;
        }
        
        /* Start capture */
        err = chronos_start_capture();
        if (err != CHRONOS_OK) {
            std::cerr << "Failed to start capture: " << chronos_strerror(err) << std::endl;
            chronos_shutdown();
            return 1;
        }
        
        running_ = true;
        
        auto start_time = std::chrono::steady_clock::now();
        int frame_count = 0;
        auto fps_time = start_time;
        
        std::cout << "\nChronos Demo - 4-Camera Synchronized Capture\n";
        std::cout << "=============================================\n";
        std::cout << "Frame Rate: " << opts_.frame_rate << " fps\n";
        std::cout << "Exposure: " << opts_.exposure_us << " us\n";
        std::cout << "Gain: " << opts_.gain_db << " dB\n";
        std::cout << "\nPress Ctrl+C to stop...\n\n";
        
        while (running_) {
            chronos_sync_frame_set_t frame_set;
            
            err = chronos_get_frame_set(&frame_set, 1000);
            if (err == CHRONOS_ERROR_TIMEOUT) {
                std::cerr << "Frame timeout!" << std::endl;
                continue;
            }
            
            if (err != CHRONOS_OK) {
                std::cerr << "Capture error: " << chronos_strerror(err) << std::endl;
                break;
            }
            
            /* Process frames on GPU */
            process_frames(&frame_set);
            
            /* Display IMU data */
            if (opts_.show_imu) {
                display_imu(&frame_set.imu);
            }
            
            /* Display frame info */
            display_frame_info(&frame_set);
            
            /* Release frame set */
            chronos_release_frame_set(&frame_set);
            
            frame_count++;
            
            /* Calculate FPS every second */
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - fps_time);
            
            if (elapsed.count() >= 1000) {
                float fps = frame_count * 1000.0f / elapsed.count();
                std::cout << "\rFPS: " << std::fixed << std::setprecision(1) << fps 
                          << "  Sync: " << frame_set.sync_sequence << "  ";
                std::cout.flush();
                
                frame_count = 0;
                fps_time = now;
            }
            
            /* Check duration */
            if (opts_.duration_sec > 0) {
                auto total = std::chrono::duration_cast<std::chrono::seconds>(now - start_time);
                if (total.count() >= opts_.duration_sec) {
                    break;
                }
            }
        }
        
        /* Print statistics */
        print_stats();
        
        /* Cleanup */
        chronos_stop_capture();
        chronos_shutdown();
        
        return 0;
    }
    
    void stop() {
        running_ = false;
    }
    
private:
    void process_frames(chronos_sync_frame_set_t* frame_set) {
        dim3 block(16, 16);
        dim3 grid((CHRONOS_FRAME_WIDTH + 15) / 16, (CHRONOS_FRAME_HEIGHT + 15) / 16);
        
        /* Convert each camera frame to 8-bit */
        for (int i = 0; i < CHRONOS_NUM_CAMERAS; i++) {
            void* cuda_ptr;
            
            if (chronos_get_cuda_ptr(&frame_set->frames[i], &cuda_ptr) == CHRONOS_OK) {
                convert_10bit_to_8bit_kernel<<<grid, block>>>(
                    (uint16_t*)cuda_ptr,
                    gpu_frames_8bit_[i],
                    CHRONOS_FRAME_WIDTH, CHRONOS_FRAME_HEIGHT,
                    CHRONOS_FRAME_WIDTH * 2,  /* 16-bit pitch */
                    CHRONOS_FRAME_WIDTH,      /* 8-bit pitch */
                    false  /* No equalization */
                );
            }
        }
        
        /* Create quad view */
        create_quad_view_kernel<<<grid, block>>>(
            gpu_frames_8bit_[0], gpu_frames_8bit_[1],
            gpu_frames_8bit_[2], gpu_frames_8bit_[3],
            gpu_quad_view_,
            CHRONOS_FRAME_WIDTH, CHRONOS_FRAME_HEIGHT, CHRONOS_FRAME_WIDTH,
            CHRONOS_FRAME_WIDTH, CHRONOS_FRAME_HEIGHT, CHRONOS_FRAME_WIDTH
        );
        
        cudaDeviceSynchronize();
        
        /* Copy to host for display (if needed) */
        if (!opts_.headless) {
            cudaMemcpy(host_display_, gpu_quad_view_,
                      CHRONOS_FRAME_WIDTH * CHRONOS_FRAME_HEIGHT,
                      cudaMemcpyDeviceToHost);
        }
    }
    
    void display_imu(const chronos_imu_data_t* imu) {
        static int counter = 0;
        
        if (++counter % 30 != 0) return;  /* Update every 30 frames */
        
        std::cout << "\n  IMU: Accel[" 
                  << std::fixed << std::setprecision(2)
                  << std::setw(7) << imu->accel[0] << ", "
                  << std::setw(7) << imu->accel[1] << ", "
                  << std::setw(7) << imu->accel[2] << "] m/s²  "
                  << "Gyro[" 
                  << std::setw(7) << imu->gyro[0] << ", "
                  << std::setw(7) << imu->gyro[1] << ", "
                  << std::setw(7) << imu->gyro[2] << "] rad/s"
                  << std::endl;
    }
    
    void display_frame_info(const chronos_sync_frame_set_t* frame_set) {
        static int counter = 0;
        
        if (++counter % 120 != 0) return;  /* Update every 120 frames */
        
        std::cout << "\n  Frame Sync: ";
        for (int i = 0; i < CHRONOS_NUM_CAMERAS; i++) {
            std::cout << "Cam" << i << ":" << frame_set->frames[i].meta.sequence << " ";
        }
        
        if (frame_set->complete) {
            std::cout << "[SYNC OK]";
        } else {
            std::cout << "[SYNC FAIL]";
        }
        std::cout << std::endl;
    }
    
    void print_stats() {
        chronos_stats_t stats;
        chronos_get_stats(&stats);
        
        std::cout << "\n\n=== Capture Statistics ===\n";
        std::cout << "Frames captured: ";
        for (int i = 0; i < CHRONOS_NUM_CAMERAS; i++) {
            std::cout << "Cam" << i << ":" << stats.frames_captured[i] << " ";
        }
        std::cout << "\nFrames dropped: ";
        for (int i = 0; i < CHRONOS_NUM_CAMERAS; i++) {
            std::cout << "Cam" << i << ":" << stats.frames_dropped[i] << " ";
        }
        std::cout << "\nSync errors: " << stats.sync_errors;
        std::cout << "\nBuffer overruns: " << stats.buffer_overruns;
        std::cout << "\nAvg latency: " << stats.avg_latency_us << " us";
        std::cout << "\nMax sync skew: " << stats.max_sync_skew_us << " us\n";
    }
    
    Options opts_;
    bool running_;
    
    uint8_t* gpu_frames_8bit_[CHRONOS_NUM_CAMERAS] = {nullptr};
    uint8_t* gpu_quad_view_ = nullptr;
    uint8_t* host_display_ = nullptr;
};

/* ============================================================================
 * Signal Handler
 * ============================================================================ */

static ChronosDemo* g_demo = nullptr;

void signal_handler(int sig) {
    (void)sig;
    std::cout << "\nStopping capture..." << std::endl;
    if (g_demo) {
        g_demo->stop();
    }
}

/* ============================================================================
 * Main
 * ============================================================================ */

void print_usage(const char* prog) {
    std::cout << "Usage: " << prog << " [options]\n\n"
              << "Options:\n"
              << "  -r, --rate <fps>      Frame rate (1-120, default: 30)\n"
              << "  -e, --exposure <us>   Exposure time in microseconds (default: 5000)\n"
              << "  -g, --gain <dB>       Analog gain in dB (default: 0.0)\n"
              << "  -d, --duration <sec>  Run duration in seconds (0=infinite)\n"
              << "  -n, --no-imu          Disable IMU display\n"
              << "  -H, --headless        Run without display\n"
              << "  -h, --help            Show this help\n";
}

int main(int argc, char** argv) {
    ChronosDemo::Options opts;
    
    static struct option long_options[] = {
        {"rate",     required_argument, 0, 'r'},
        {"exposure", required_argument, 0, 'e'},
        {"gain",     required_argument, 0, 'g'},
        {"duration", required_argument, 0, 'd'},
        {"no-imu",   no_argument,       0, 'n'},
        {"headless", no_argument,       0, 'H'},
        {"help",     no_argument,       0, 'h'},
        {0, 0, 0, 0}
    };
    
    int c;
    while ((c = getopt_long(argc, argv, "r:e:g:d:nHh", long_options, nullptr)) != -1) {
        switch (c) {
        case 'r':
            opts.frame_rate = atoi(optarg);
            break;
        case 'e':
            opts.exposure_us = atoi(optarg);
            break;
        case 'g':
            opts.gain_db = atof(optarg);
            break;
        case 'd':
            opts.duration_sec = atoi(optarg);
            break;
        case 'n':
            opts.show_imu = false;
            break;
        case 'H':
            opts.headless = true;
            break;
        case 'h':
            print_usage(argv[0]);
            return 0;
        default:
            print_usage(argv[0]);
            return 1;
        }
    }
    
    /* Validate options */
    if (opts.frame_rate < 1 || opts.frame_rate > 120) {
        std::cerr << "Invalid frame rate. Must be 1-120.\n";
        return 1;
    }
    
    /* Setup signal handler */
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    /* Run demo */
    ChronosDemo demo(opts);
    g_demo = &demo;
    
    return demo.run();
}
