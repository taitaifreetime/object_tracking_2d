/**
 * @file laser_scan_filter.hpp
 * @author tomine (tomine@aisl.cs.tut.ac.jp)
 * @brief utilities for laser scan pre-processer
 * @version 0.1.0
 * @date 2024-07-10
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef LASER_SCAN_FILTER_HPP
#define LASER_SCAN_FILTER_HPP

#include "sensor_msgs/msg/laser_scan.hpp"

class LaserScanFilter{
    public:

        /**
         * @brief pre-processed laser scan type
         * 
         */
        struct PreProcScan{
            int index; // original index in the original laserscan
            float range; 
            float angle;
        };
        
        /**
         * @brief Construct a new Laser Scan Filter object
         * 
         * @param range_max max distance from lasescan frame to obstacle to be detected
         * @param range_min min distance from lasescan frame to obstacle to be detected
         * @param angle_max max angle in a horizontal direction to obstacle to be detected
         * @param angle_min min angle in a horizontal direction to obstacle to be detected
         * @param ma_interval ave of ma_interval samples for moving average pre-process
         */
        LaserScanFilter(
            float range_max, float range_min, 
            float angle_max, float angle_min, 
            int ma_interval
        );
        ~LaserScanFilter();

        /**
         * @brief filter laserscan
         * 
         * @param scan original laserscan msg, const
         * @param cropped_finite_scans finite laserscan, not const
         */
        void EliminateInf(const sensor_msgs::msg::LaserScan &scan, std::vector<PreProcScan> &cropped_finite_scans) const;

        /**
         * @brief extend finite laserscan by adding a first part of the scan to the end of the scan 
         * 
         * @param extended_scans extended scan, not const
         */
        void ExtendScan(std::vector<PreProcScan> &extended_scans) const;

        /**
         * @brief make extended laserscan smooth by moving average
         * 
         * @param ma_scan laserscan after moving average, not const
         */
        void MovingAve(std::vector<PreProcScan> &ma_scan) const;

        /**
         * @brief pre-process
         * 
         * @param original_scan original laserscan, const
         * @param preproc_scan pre-processed laserscan, not const
         */
        void PreProc(const sensor_msgs::msg::LaserScan &original_scan, std::vector<PreProcScan> &preproc_scan) const;

    private:
        float range_max_, range_min_;
        float angle_max_, angle_min_;
        int ma_interval_;
        int ma_interval_2_; // ma_interval_/2
};

#endif // LASER_SCAN_FILTER_HPP