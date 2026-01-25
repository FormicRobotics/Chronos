/*
 * Chronos Synchronization Test Application
 *
 * Copyright (C) 2025 Chronos Project
 *
 * Validates sub-microsecond synchronization accuracy and trigger skew
 */

#include <iostream>
#include <iomanip>
#include <vector>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <signal.h>

#include "chronos_capture.h"

/* ============================================================================
 * Statistics Calculator
 * ============================================================================ */

class Stats {
public:
    void add(double value) {
        values_.push_back(value);
    }
    
    void clear() {
        values_.clear();
    }
    
    size_t count() const {
        return values_.size();
    }
    
    double mean() const {
        if (values_.empty()) return 0.0;
        return std::accumulate(values_.begin(), values_.end(), 0.0) / values_.size();
    }
    
    double stddev() const {
        if (values_.size() < 2) return 0.0;
        double m = mean();
        double sq_sum = 0.0;
        for (double v : values_) {
            sq_sum += (v - m) * (v - m);
        }
        return std::sqrt(sq_sum / (values_.size() - 1));
    }
    
    double min() const {
        if (values_.empty()) return 0.0;
        return *std::min_element(values_.begin(), values_.end());
    }
    
    double max() const {
        if (values_.empty()) return 0.0;
        return *std::max_element(values_.begin(), values_.end());
    }
    
    double percentile(double p) const {
        if (values_.empty()) return 0.0;
        std::vector<double> sorted = values_;
        std::sort(sorted.begin(), sorted.end());
        size_t idx = static_cast<size_t>(p / 100.0 * (sorted.size() - 1));
        return sorted[idx];
    }
    
private:
    std::vector<double> values_;
};

/* ============================================================================
 * Sync Test Class
 * ============================================================================ */

class SyncTest {
public:
    struct Options {
        int frame_count = 1000;
        int frame_rate = 120;
        bool verbose = false;
    };
    
    SyncTest(const Options& opts) : opts_(opts), running_(true) {}
    
    int run() {
        /* Initialize */
        chronos_error_t err = chronos_init();
        if (err != CHRONOS_OK) {
            std::cerr << "Failed to initialize: " << chronos_strerror(err) << std::endl;
            return 1;
        }
        
        /* Configure for maximum frame rate */
        chronos_config_t config = {};
        config.frame_rate = opts_.frame_rate;
        config.exposure_us = 1000;  /* Short exposure for timing tests */
        config.gain_db = 0.0f;
        config.external_trigger = true;
        config.enable_imu = true;
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
        
        std::cout << "\nChronos Synchronization Test\n";
        std::cout << "============================\n";
        std::cout << "Target frame rate: " << opts_.frame_rate << " fps\n";
        std::cout << "Test frames: " << opts_.frame_count << "\n\n";
        std::cout << "Collecting data...\n\n";
        
        /* Collect timing data */
        Stats frame_skew;           /* Inter-camera skew */
        Stats frame_interval;       /* Frame-to-frame interval */
        Stats imu_camera_offset;    /* IMU to camera timestamp offset */
        
        uint64_t prev_timestamp = 0;
        int collected = 0;
        
        while (running_ && collected < opts_.frame_count) {
            chronos_sync_frame_set_t frame_set;
            
            err = chronos_get_frame_set(&frame_set, 1000);
            if (err == CHRONOS_ERROR_TIMEOUT) {
                std::cerr << "Timeout waiting for frame\n";
                continue;
            }
            
            if (err != CHRONOS_OK) {
                std::cerr << "Error: " << chronos_strerror(err) << std::endl;
                break;
            }
            
            if (!frame_set.complete) {
                chronos_release_frame_set(&frame_set);
                continue;  /* Skip incomplete frame sets */
            }
            
            /* Calculate inter-camera skew */
            uint64_t min_ts = UINT64_MAX;
            uint64_t max_ts = 0;
            
            for (int i = 0; i < CHRONOS_NUM_CAMERAS; i++) {
                uint64_t ts = frame_set.frames[i].meta.timestamp_ns;
                if (ts < min_ts) min_ts = ts;
                if (ts > max_ts) max_ts = ts;
            }
            
            double skew_us = (max_ts - min_ts) / 1000.0;
            frame_skew.add(skew_us);
            
            /* Calculate frame interval */
            if (prev_timestamp > 0) {
                double interval_us = (frame_set.frames[0].meta.timestamp_ns - prev_timestamp) / 1000.0;
                frame_interval.add(interval_us);
            }
            prev_timestamp = frame_set.frames[0].meta.timestamp_ns;
            
            /* Calculate IMU-camera offset */
            if (frame_set.imu.timestamp_ns > 0) {
                double offset_us = ((int64_t)frame_set.imu.timestamp_ns - 
                                   (int64_t)frame_set.frames[0].meta.timestamp_ns) / 1000.0;
                imu_camera_offset.add(offset_us);
            }
            
            /* Verbose output */
            if (opts_.verbose && collected % 100 == 0) {
                std::cout << "Frame " << collected << ": skew=" << std::fixed 
                          << std::setprecision(2) << skew_us << " us\n";
            }
            
            chronos_release_frame_set(&frame_set);
            collected++;
            
            /* Progress */
            if (collected % 100 == 0) {
                std::cout << "\rProgress: " << collected << "/" << opts_.frame_count 
                          << " (" << (collected * 100 / opts_.frame_count) << "%)";
                std::cout.flush();
            }
        }
        
        std::cout << "\n\n";
        
        /* Print results */
        print_results(frame_skew, frame_interval, imu_camera_offset);
        
        /* Cleanup */
        chronos_stop_capture();
        chronos_shutdown();
        
        return 0;
    }
    
    void stop() {
        running_ = false;
    }
    
private:
    void print_results(const Stats& skew, const Stats& interval, 
                       const Stats& imu_offset) {
        double expected_interval_us = 1000000.0 / opts_.frame_rate;
        
        std::cout << "=== SYNCHRONIZATION TEST RESULTS ===\n\n";
        
        std::cout << "Inter-Camera Skew (all 4 cameras):\n";
        std::cout << "  Samples:    " << skew.count() << "\n";
        std::cout << "  Mean:       " << std::fixed << std::setprecision(3) 
                  << skew.mean() << " us\n";
        std::cout << "  Std Dev:    " << skew.stddev() << " us\n";
        std::cout << "  Min:        " << skew.min() << " us\n";
        std::cout << "  Max:        " << skew.max() << " us\n";
        std::cout << "  99th %ile:  " << skew.percentile(99) << " us\n";
        
        bool skew_pass = skew.percentile(99) < 100.0;  /* <100us requirement */
        std::cout << "  RESULT:     " << (skew_pass ? "PASS" : "FAIL") 
                  << " (requirement: <100 us)\n\n";
        
        std::cout << "Frame Interval (frame-to-frame timing):\n";
        std::cout << "  Expected:   " << expected_interval_us << " us\n";
        std::cout << "  Mean:       " << interval.mean() << " us\n";
        std::cout << "  Std Dev:    " << interval.stddev() << " us\n";
        std::cout << "  Min:        " << interval.min() << " us\n";
        std::cout << "  Max:        " << interval.max() << " us\n";
        
        double interval_error = std::abs(interval.mean() - expected_interval_us);
        bool interval_pass = interval_error < (expected_interval_us * 0.01);  /* <1% error */
        std::cout << "  Error:      " << (interval_error / expected_interval_us * 100) << "%\n";
        std::cout << "  RESULT:     " << (interval_pass ? "PASS" : "FAIL") 
                  << " (requirement: <1% error)\n\n";
        
        if (imu_offset.count() > 0) {
            std::cout << "IMU-Camera Synchronization:\n";
            std::cout << "  Mean Offset: " << imu_offset.mean() << " us\n";
            std::cout << "  Std Dev:     " << imu_offset.stddev() << " us\n";
            std::cout << "  Max Offset:  " << std::max(std::abs(imu_offset.min()), 
                                                        std::abs(imu_offset.max())) << " us\n\n";
        }
        
        std::cout << "=== OVERALL: ";
        if (skew_pass && interval_pass) {
            std::cout << "ALL TESTS PASSED ===\n";
        } else {
            std::cout << "SOME TESTS FAILED ===\n";
        }
    }
    
    Options opts_;
    bool running_;
};

/* ============================================================================
 * Main
 * ============================================================================ */

static SyncTest* g_test = nullptr;

void signal_handler(int sig) {
    (void)sig;
    if (g_test) g_test->stop();
}

int main(int argc, char** argv) {
    SyncTest::Options opts;
    
    /* Parse arguments */
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "-n" && i + 1 < argc) {
            opts.frame_count = std::atoi(argv[++i]);
        } else if (arg == "-r" && i + 1 < argc) {
            opts.frame_rate = std::atoi(argv[++i]);
        } else if (arg == "-v") {
            opts.verbose = true;
        } else if (arg == "-h" || arg == "--help") {
            std::cout << "Usage: " << argv[0] << " [options]\n\n"
                      << "Options:\n"
                      << "  -n <count>    Number of frames to test (default: 1000)\n"
                      << "  -r <fps>      Frame rate (default: 120)\n"
                      << "  -v            Verbose output\n"
                      << "  -h            Show this help\n";
            return 0;
        }
    }
    
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    SyncTest test(opts);
    g_test = &test;
    
    return test.run();
}
