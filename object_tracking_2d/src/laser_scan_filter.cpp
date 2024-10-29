#include "object_tracking_2d/laser_scan_filter.hpp"
#include <cmath>
#include "rclcpp/rclcpp.hpp"

LaserScanFilter::LaserScanFilter(
    float range_max, 
    float range_min, 
    float angle_max, 
    float angle_min,
    int ma_interval
):
    range_max_(range_max), 
    range_min_(range_min), 
    angle_max_(angle_max), 
    angle_min_(angle_min), 
    ma_interval_(ma_interval),
    ma_interval_2_(ma_interval_ / 2)
{}
LaserScanFilter::~LaserScanFilter(){}

void LaserScanFilter::EliminateInf(const sensor_msgs::msg::LaserScan &scan, std::vector<LaserScanFilter::PreProcScan> &cropped_finite_scans) const
{
    int num_scans = scan.ranges.size();
    float angle_increment = scan.angle_increment;
    for (int i=0; i<num_scans; i++)
    {
        float range = scan.ranges[i];
        float angle = i * angle_increment;
        if (
            std::isfinite(range)
            && range < range_max_ 
            && range > range_min_
            && angle > angle_min_
            && angle < angle_max_
        )
        {
            PreProcScan preproc_scan;
            preproc_scan.index = i;
            preproc_scan.range = range;
            preproc_scan.angle = angle;
            cropped_finite_scans.push_back(preproc_scan);
        }
    }
}

void LaserScanFilter::ExtendScan(std::vector<LaserScanFilter::PreProcScan> &extended_scans) const
{
    int until = int(extended_scans.size() / 200);
    for (int i=0; i<until; i++)
    {
        extended_scans.push_back(extended_scans[i]);
    } 
}

void LaserScanFilter::MovingAve(std::vector<LaserScanFilter::PreProcScan> &ma_scans) const
{  
    std::vector<PreProcScan> scans;
    int num_scans = ma_scans.size();
    for (int i=0; i<num_scans-ma_interval_2_; i+=ma_interval_2_){
        float ave_range = 0.0;
        float ave_angle = ma_scans[i+ma_interval_2_].angle;
        for (int j=0; j<ma_interval_; j++){
            ave_range += ma_scans[i+j].range;
        }
        ave_range/=float(ma_interval_);
        
        PreProcScan preproc_scan;
        preproc_scan.index = i+ma_interval_2_;
        preproc_scan.range = ave_range;
        preproc_scan.angle = ave_angle;
        scans.push_back(preproc_scan);
    }
    ma_scans = scans;
}

void LaserScanFilter::PreProc(const sensor_msgs::msg::LaserScan &original_scan, std::vector<LaserScanFilter::PreProcScan> &preproc_scans) const
{
    EliminateInf(original_scan, preproc_scans);
    // ExtendScan(preproc_scans);
    MovingAve(preproc_scans);
}