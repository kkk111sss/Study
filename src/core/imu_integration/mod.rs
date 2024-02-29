// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright © 2021-2022 Adrian <adrian.eddy at gmail>

mod complementary_v2;
mod complementary;
mod vqf;

use std::{collections::BTreeMap, io::Write};
use nalgebra::*;
use super::gyro_source::{TimeIMU, Quat64, TimeQuat};
use ahrs::{Ahrs, Madgwick, Mahony};

// TODO: Magnetometer calculations are disabled in Complementary and VQF. Figure out what's wrong and enable them

pub trait GyroIntegrator {
    fn integrate(imu_data: &[TimeIMU], duration_ms: f64) -> TimeQuat;
}

pub struct QuaternionConverter { }
pub struct ComplementaryIntegrator { }
pub struct VQFIntegrator { }
pub struct SimpleGyroIntegrator { }
pub struct SimpleGyroAccelIntegrator { }
pub struct MahonyIntegrator { }
pub struct MadgwickIntegrator { }

// const RAD2DEG: f64 = 180.0 / std::f64::consts::PI;
const DEG2RAD: f64 = std::f64::consts::PI / 180.0;

impl QuaternionConverter {
    pub fn convert(method: i32, org_quaternions: &TimeQuat, image_orientations : &TimeQuat, imu_data: &[TimeIMU], duration_ms: f64) -> TimeQuat {
        let integrated_quats = match method {
            0 => ComplementaryIntegrator  ::integrate(imu_data, duration_ms),
            1 => VQFIntegrator            ::integrate(imu_data, duration_ms),
            2 => SimpleGyroAccelIntegrator::integrate(imu_data, duration_ms),
            3 => MahonyIntegrator         ::integrate(imu_data, duration_ms),
            4 => MadgwickIntegrator       ::integrate(imu_data, duration_ms),
            _ => VQFIntegrator            ::integrate(imu_data, duration_ms),
        };
        let mut boost = 1;
        let mut ret : TimeQuat = BTreeMap::new();
        let mut corr_sm = UnitQuaternion::identity();
        for (&org_ts, &org_quat) in org_quaternions {
            let n_quat = integrated_quats.range(org_ts..).next().map(|x|*x.1).unwrap_or(UnitQuaternion::identity());
            let io_quat = image_orientations.range(org_ts..).next().map(|x|*x.1).unwrap_or(UnitQuaternion::identity());
            let corr = n_quat * (org_quat * io_quat.inverse()).inverse();
            corr_sm = corr_sm.slerp(&corr, if boost > 0 { boost -= 1; 1.0 } else { 0.005 });
            ret.insert(org_ts, corr_sm * org_quat);
        }
        ret
    }
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

use complementary_v2::ComplementaryFilterV2;

impl GyroIntegrator for ComplementaryIntegrator {
    fn integrate(imu_data: &[TimeIMU], duration_ms: f64) -> TimeQuat {
        if imu_data.is_empty() { return BTreeMap::new(); }
        let mut quats = BTreeMap::new();
        let sample_time_ms = duration_ms / imu_data.len() as f64;

        let mut f = ComplementaryFilterV2::default();
        // Limit initial settle time for short videos
        f.set_initial_settle_time((duration_ms / 1000.0 * 0.05).min(2.0));
        //f.set_orientation(init_pos_q.scalar(), -init_pos_q.vector()[0], -init_pos_q.vector()[1], -init_pos_q.vector()[2]);
        let mut counter = 0;
        let mut prev_time = imu_data[0].timestamp_ms - sample_time_ms;
        for v in imu_data {
            if let Some(g) = v.gyro.as_ref() {
                let mut a = v.accl.unwrap_or_default();
                if a[0].abs() == 0.0 && a[1].abs() == 0.0 && a[2].abs() == 0.0 { a[0] += 0.0000001; }
                let acc = Vector3::new(-a[1], a[0], a[2]);
                // log::info!("acc norm: {}", acc.norm());

                /*if let Some(m) = v.magn.as_ref() {
                    if let Some(magn) = Vector3::new(-m[1], m[0], m[2]).try_normalize(0.0) {
                        f.update_mag(acc[0], acc[1], acc[2],
                            -g[1] * DEG2RAD, g[0] * DEG2RAD, g[2] * DEG2RAD,
                            magn[0], magn[1], magn[2],
                            (v.timestamp_ms - prev_time) / 1000.0);
                    }
                } else */{
                    counter += 1;
                    if counter % 20 == 0 {
                        //println!("{:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}", v.timestamp_ms, acc[0], acc[1], acc[2], -g[1] * DEG2RAD, g[0] * DEG2RAD, g[2] * DEG2RAD);
                    }
                    f.update(acc[0], acc[1], acc[2],
                        -g[1] * DEG2RAD, g[0] * DEG2RAD, g[2] * DEG2RAD,
                        (v.timestamp_ms - prev_time) / 1000.0);
                }
                let x = f.get_orientation();
                quats.insert((v.timestamp_ms * 1000.0) as i64, Quat64::from_quaternion(Quaternion::from_parts(x.0, Vector3::new(x.1, x.2, x.3))));

                prev_time = v.timestamp_ms;
            }
        }

        quats
    }
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

impl GyroIntegrator for VQFIntegrator {
    // VQF方法实现
    fn integrate(imu_data: &[TimeIMU],      // IMU数据
                 duration_ms: f64)          // IMU数据总时长
                 -> TimeQuat {              // 返回积分得到的四元数序列
        // 检查IMU数据是否非空
        if imu_data.is_empty() { return BTreeMap::new(); }
        // 初始化方向四元数列表
        let mut out_quats = BTreeMap::new();
        // 采样时间间隔 = IMU数据总时长 除以 (IMU数据个数*1000)?
        let sample_time = duration_ms / (imu_data.len() * 1000) as f64;
        // 采样点个数
        let num_samples = imu_data.len();
        // 创建采样点个陀螺仪角速度向量
        let mut gyr = Vec::with_capacity(num_samples*3);
        // 创建采样点个加速度计加速度向量
        let mut acc = Vec::with_capacity(num_samples*3);
        // 创建采样点个磁力计磁场强度向量
        let mut mag = Vec::with_capacity(num_samples*3);
        // 创建采样点个四元数
        let mut quat = Vec::with_capacity(num_samples*4);
        // 历遍所有IMU数据
        for v in imu_data {
            // 获取陀螺仪数据
            let g = v.gyro.unwrap_or_default();
            // zero mag or acc (default) is ignored by VQF
            // 获取加速度计数据
            let a = v.accl.unwrap_or_default();
            // 获取磁力计数据，这里没用到直接赋0
            let m = [0.0, 0.0, 0.0]; // v.magn.unwrap_or_default();
            // 调整陀螺仪读数顺序，角度转换为弧度
            gyr.extend([-g[1] * DEG2RAD, g[0] * DEG2RAD, g[2] * DEG2RAD]);
            // 调整加速度计读数顺序
            acc.extend([-a[1], a[0], a[2]]);
            // 调整磁力计读数顺序
            mag.extend([-m[1], m[0], m[2]]);
            // 初始化四元数方向
            quat.extend([1.0, 0.0, 0.0, 0.0]);
        }

        // Tweak parameters here, see parameter descriptions: https://github.com/dlaidig/vqf/blob/main/vqf/cpp/vqf.hpp#L37
        let params = vqf::VQFParams {
            tau_acc: 40.0,          // 加速度计低通滤波，值越大表示对陀螺仪测量值越信任，默认3
            tau_mag: 40.0,          // 磁力计更新周期，值越大表示对陀螺仪测量值越信任，默认9
            ..Default::default()
        };
        // vqf运动估计
        vqf::offline_vqf(&gyr, &acc, Some(&mag), num_samples, sample_time, params, &mut Vec::new(), Some(&mut quat), Some(&mut Vec::new()), &mut Vec::new(), None, None, Some(&mut Vec::new()));
        drop(gyr); drop(acc); drop(mag);

        // by lxq
        // 创建文件
        let mut file = std::fs::File::create("E:/视频防抖/data/imu_data/135852/gyroflow_vqf_q.txt").expect("create failed");

        for (i, v) in imu_data.iter().enumerate() {
            out_quats.insert((v.timestamp_ms * 1000.0) as i64, Quat64::from_quaternion(Quaternion::from_parts(quat[i*4], Vector3::new(quat[i*4+1], quat[i*4+2], quat[i*4+3]))));
            // by lxq
            //println!("{} {} {} {} {}", i*2, quat[i*4], quat[i*4+1], quat[i*4+2], quat[i*4+3]); // 打印数组
            file.write_all(format!("{} {} {} {} {}\n", i*2, quat[i*4], quat[i*4+1], quat[i*4+2], quat[i*4+3]).as_bytes());
        }
        // println!("out_quats is {:?}", out_quats); // 打印结构体
        out_quats
    }
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

impl GyroIntegrator for SimpleGyroIntegrator {
    fn integrate(imu_data: &[TimeIMU], duration_ms: f64) -> TimeQuat {
        if imu_data.is_empty() { return BTreeMap::new(); }
        let mut quats = BTreeMap::new();
        let mut orientation = UnitQuaternion::from_euler_angles(std::f64::consts::FRAC_PI_2, 0.0, 0.0);

        let sample_time_ms = duration_ms / imu_data.len() as f64;
        let mut prev_time = imu_data[0].timestamp_ms - sample_time_ms;

        for v in imu_data {
            if let Some(g) = v.gyro.as_ref() {
                let omega = Vector3::new(-g[1], g[0], g[2]) * (std::f64::consts::PI / 180.0);

                // calculate rotation quaternion
                let dt = (v.timestamp_ms - prev_time) / 1000.0;
                let delta_q = UnitQuaternion::from_scaled_axis(omega * dt);

                // rotate orientation by this quaternion
                orientation = Quat64::from_quaternion(orientation.quaternion() * delta_q.quaternion());

                quats.insert((v.timestamp_ms * 1000.0) as i64, orientation);

                prev_time = v.timestamp_ms;
            }
        }

        quats
    }
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

impl GyroIntegrator for SimpleGyroAccelIntegrator {
    fn integrate(imu_data: &[TimeIMU], duration_ms: f64) -> TimeQuat {
        if imu_data.is_empty() { return BTreeMap::new(); }
        let mut quats = BTreeMap::new();
        let mut orientation = UnitQuaternion::from_euler_angles(std::f64::consts::FRAC_PI_2, 0.0, 0.0);

        let sample_time_ms = duration_ms / imu_data.len() as f64;
        let mut prev_time = imu_data[0].timestamp_ms - sample_time_ms;
        let start_time = prev_time;

        for v in imu_data {
            if let Some(g) = v.gyro.as_ref() {
                let mut omega = Vector3::new(-g[1], g[0], g[2]) * (std::f64::consts::PI / 180.0);

                let a = v.accl.unwrap_or_default();
                let acc = Vector3::new(-a[1], a[0], a[2]).try_normalize(0.0).unwrap_or_default();
                let g = acc.norm();
                if (0.9..1.1).contains(&g) {
                    let acc_world_vec = orientation * acc;
                    let correction_world = acc_world_vec.cross(&Vector3::new(0.0, 0.0, 1.0));

                    // high weight for first 1.5s to "lock" it
                    let weight = if v.timestamp_ms - start_time < 15000.0 { 10.0 } else { 0.6 };
                    let correction_body = weight * (orientation.conjugate() * correction_world);
                    omega += correction_body;
                }

                // calculate rotation quaternion
                let dt = (v.timestamp_ms - prev_time) / 1000.0;
                let delta_q = UnitQuaternion::from_scaled_axis(omega * dt);

                // rotate orientation by this quaternion
                orientation = Quat64::from_quaternion(orientation.quaternion() * delta_q.quaternion());

                quats.insert((v.timestamp_ms * 1000.0) as i64, orientation);

                prev_time = v.timestamp_ms;
            }
        }

        quats
    }
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

impl GyroIntegrator for MahonyIntegrator {
    fn integrate(imu_data: &[TimeIMU], duration_ms: f64) -> TimeQuat {
        if imu_data.is_empty() { return BTreeMap::new(); }

        let mut quats = BTreeMap::new();
        let init_pos = UnitQuaternion::from_euler_angles(std::f64::consts::FRAC_PI_2, 0.0, 0.0);
        let sample_time_s = duration_ms / 1000.0 / imu_data.len() as f64;

        let mut ahrs = Mahony::new_with_quat(sample_time_s, 0.5, 0.0, init_pos);
        let mut prev_time = imu_data[0].timestamp_ms - sample_time_s;
        for v in imu_data {
            if let Some(g) = v.gyro.as_ref() {
                let gyro = Vector3::new(-g[1], g[0], g[2]) * (std::f64::consts::PI / 180.0);
                let mut a = v.accl.unwrap_or_default();
                if a[0].abs() == 0.0 && a[1].abs() == 0.0 && a[2].abs() == 0.0 { a[0] += 0.0000001; }
                let accl = Vector3::new(-a[1], a[0], a[2]);

                *ahrs.sample_period_mut() = (v.timestamp_ms - prev_time) / 1000.0;

                if let Some(m) = v.magn.as_ref() {
                    let magn = Vector3::new(-m[1], m[0], m[2]);

                    match ahrs.update(&gyro, &accl, &magn) {
                        Ok(quat) => { quats.insert((v.timestamp_ms * 1000.0) as i64, *quat); },
                        Err(e) => log::warn!("Invalid data! {} Gyro: [{}, {}, {}] Accel: [{}, {}, {}] Magn: [{}, {}, {}]", e, gyro[0], gyro[1], gyro[2], accl[0], accl[1], accl[2], magn[0], magn[1], magn[2])
                    }
                } else {
                    match ahrs.update_imu(&gyro, &accl) {
                        Ok(quat) => { quats.insert((v.timestamp_ms * 1000.0) as i64, *quat); },
                        Err(e) => log::warn!("Invalid data! {} Gyro: [{}, {}, {}] Accel: [{}, {}, {}]", e, gyro[0], gyro[1], gyro[2], accl[0], accl[1], accl[2])
                    }
                }
                prev_time = v.timestamp_ms;
            }
        }

        quats
    }
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

impl GyroIntegrator for MadgwickIntegrator {
    fn integrate(imu_data: &[TimeIMU], duration_ms: f64) -> TimeQuat {
        if imu_data.is_empty() { return BTreeMap::new(); }

        let mut quats = BTreeMap::new();
        let init_pos = UnitQuaternion::from_euler_angles(std::f64::consts::FRAC_PI_2, 0.0, 0.0);
        let sample_time_s = duration_ms / 1000.0 / imu_data.len() as f64;

        let mut ahrs = Madgwick::new_with_quat(sample_time_s, 0.02, init_pos);
        let mut prev_time = imu_data[0].timestamp_ms - sample_time_s;
        for v in imu_data {
            if let Some(g) = v.gyro.as_ref() {
                let gyro = Vector3::new(-g[1], g[0], g[2]) * (std::f64::consts::PI / 180.0);
                let mut a = v.accl.unwrap_or_default();
                if a[0].abs() == 0.0 && a[1].abs() == 0.0 && a[2].abs() == 0.0 { a[0] += 0.0000001; }
                let accl = Vector3::new(-a[1], a[0], a[2]);

                *ahrs.sample_period_mut() = (v.timestamp_ms - prev_time) / 1000.0;

                if let Some(m) = v.magn.as_ref() {
                    let magn = Vector3::new(-m[1], m[0], m[2]);

                    match ahrs.update(&gyro, &accl, &magn) {
                        Ok(quat) => { quats.insert((v.timestamp_ms * 1000.0) as i64, *quat); },
                        Err(e) => log::warn!("Invalid data! {} Gyro: [{}, {}, {}] Accel: [{}, {}, {}] Magn: [{}, {}, {}]", e, gyro[0], gyro[1], gyro[2], accl[0], accl[1], accl[2], magn[0], magn[1], magn[2])
                    }
                } else {
                    match ahrs.update_imu(&gyro, &accl) {
                        Ok(quat) => { quats.insert((v.timestamp_ms * 1000.0) as i64, *quat); },
                        Err(e) => log::warn!("Invalid data! {} Gyro: [{}, {}, {}] Accel: [{}, {}, {}]", e, gyro[0], gyro[1], gyro[2], accl[0], accl[1], accl[2])
                    }
                }
                prev_time = v.timestamp_ms;
            }
        }

        quats
    }
}
