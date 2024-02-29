// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright © 2021-2022 Aphobius

// 1. Calculate velocity for each quaternion
// 2. Smooth the velocities
// 3. Multiply max velocity (500 deg/s) with slider value
// 4. Perform plain 3D smoothing with varying alpha, where each alpha is interpolated between 1s smoothness at 0 velocity, 0.1s smoothness at max velocity and extrapolated above that
// 5. This way, low velocities are smoothed using 1s smoothness, but high velocities are smoothed using 0.1s smoothness at max velocity (500 deg/s multiplied by slider) and gradually lower smoothness above that
// 6. Calculate distance from smoothed quaternions to raw quaternions
// 7. Normalize distance and set everything bellow 0.5 to 0.0
// 8. Smooth distance
// 9. Normalize distance again and change range to 0.5 - 1.0
// 10. Perform plain 3D smoothing, on the last smoothed quaternions, with varying alpha, interpolated between 1s and 0.1s smoothness based on previously calculated velocity multiplied by the distance

use std::collections::BTreeMap;

use super::*;
use crate::gyro_source::TimeQuat;
use nalgebra::*;
use crate::Quat64;
use crate::keyframes::*;

#[derive(Clone)]
pub struct DefaultAlgo {
    pub smoothness: f64,
    pub smoothness_pitch: f64,
    pub smoothness_yaw: f64,
    pub smoothness_roll: f64,
    pub per_axis: bool,
    pub second_pass: bool,
    pub trim_range_only: bool,
    pub max_smoothness: f64,
    pub alpha_0_1s: f64,
}

impl Default for DefaultAlgo {
    fn default() -> Self { Self {
        smoothness: 0.5,
        smoothness_pitch: 0.5,
        smoothness_yaw: 0.5,
        smoothness_roll: 0.5,
        per_axis: false,
        second_pass: true,
        trim_range_only: false,
        max_smoothness: 1.0,
        alpha_0_1s: 0.1
    } }
}

impl SmoothingAlgorithm for DefaultAlgo {
    fn get_name(&self) -> String { "Default".to_owned() }

    fn set_parameter(&mut self, name: &str, val: f64) {
        match name {
            "smoothness"       => self.smoothness = val,
            "smoothness_pitch" => self.smoothness_pitch = val,
            "smoothness_yaw"   => self.smoothness_yaw = val,
            "smoothness_roll"  => self.smoothness_roll = val,
            "per_axis"         => self.per_axis = val > 0.1,
            // "second_pass"      => self.second_pass = val > 0.1,
            "trim_range_only"  => self.trim_range_only = val > 0.1,
            "max_smoothness"   => self.max_smoothness = val,
            "alpha_0_1s"       => self.alpha_0_1s = val,
            _ => log::error!("Invalid parameter name: {}", name)
        }
    }
    fn get_parameter(&self, name: &str) -> f64 {
        match name {
            "smoothness"       => self.smoothness,
            "smoothness_pitch" => self.smoothness_pitch,
            "smoothness_yaw"   => self.smoothness_yaw,
            "smoothness_roll"  => self.smoothness_roll,
            "per_axis"         => if self.per_axis { 1.0 } else { 0.0 },
            // "second_pass"      => if self.second_pass { 1.0 } else { 0.0 },
            "trim_range_only"  => if self.trim_range_only { 1.0 } else { 0.0 },
            "max_smoothness"   => self.max_smoothness,
            "alpha_0_1s"       => self.alpha_0_1s,
            _ => 0.0
        }
    }

    fn get_parameters_json(&self) -> serde_json::Value {
        serde_json::json!([
            {
                "name": "smoothness",
                "description": "Smoothness",
                "type": "SliderWithField",
                "from": 0.001,
                "to": 1.0,
                "value": self.smoothness,
                "default": 0.5,
                "unit": "",
                "precision": 3,
                "keyframe": "SmoothingParamSmoothness"
            },
            {
                "name": "smoothness_pitch",
                "description": "Pitch smoothness",
                "type": "SliderWithField",
                "from": 0.001,
                "to": 1.0,
                "value": self.smoothness_pitch,
                "default": 0.5,
                "unit": "",
                "precision": 3,
                "keyframe": "SmoothingParamPitch"
            },
            {
                "name": "smoothness_yaw",
                "description": "Yaw smoothness",
                "type": "SliderWithField",
                "from": 0.001,
                "to": 1.0,
                "value": self.smoothness_yaw,
                "default": 0.5,
                "unit": "",
                "precision": 3,
                "keyframe": "SmoothingParamYaw"
            },
            {
                "name": "smoothness_roll",
                "description": "Roll smoothness",
                "type": "SliderWithField",
                "from": 0.001,
                "to": 1.0,
                "value": self.smoothness_roll,
                "default": 0.5,
                "unit": "",
                "precision": 3,
                "keyframe": "SmoothingParamRoll"
            },
            {
                "name": "per_axis",
                "description": "Per axis",
                "advanced": true,
                "type": "CheckBox",
                "default": self.per_axis,
                "value": if self.per_axis { 1.0 } else { 0.0 },
                "custom_qml": "Connections { function onCheckedChanged() {
                    const checked = root.getParamElement('per_axis').checked;
                    root.getParamElement('smoothness-label').visible = !checked;
                    root.getParamElement('smoothness_pitch-label').visible = checked;
                    root.getParamElement('smoothness_yaw-label').visible = checked;
                    root.getParamElement('smoothness_roll-label').visible = checked;
                }}"
            },
            /*{
                "name": "second_pass",
                "description": "Second smoothing pass",
                "advanced": true,
                "type": "CheckBox",
                "default": self.second_pass,
                "value": if self.second_pass { 1.0 } else { 0.0 },
            },*/
            {
                "name": "trim_range_only",
                "description": "Only within trim range",
                "advanced": true,
                "type": "CheckBox",
                "default": self.trim_range_only,
                "value": if self.trim_range_only { 1.0 } else { 0.0 },
            },
            {
                "name": "max_smoothness",
                "description": "Max smoothness",
                "advanced": true,
                "type": "SliderWithField",
                "from": 0.1,
                "to": 5.0,
                "value": self.max_smoothness,
                "default": 1.0,
                "precision": 3,
                "unit": "s",
                "keyframe": "SmoothingParamTimeConstant"
            },
            {
                "name": "alpha_0_1s",
                "description": "Max smoothness at high velocity",
                "advanced": true,
                "type": "SliderWithField",
                "from": 0.01,
                "to": 1.0,
                "value": self.alpha_0_1s,
                "default": 0.1,
                "precision": 3,
                "unit": "s",
                "keyframe": "SmoothingParamTimeConstant2"
            }
        ])
    }

    fn get_status_json(&self) -> serde_json::Value {
        serde_json::json!([])
    }

    fn get_checksum(&self) -> u64 {
        let mut hasher = std::collections::hash_map::DefaultHasher::new();
        hasher.write_u64(self.smoothness.to_bits());
        hasher.write_u64(self.smoothness_pitch.to_bits());
        hasher.write_u64(self.smoothness_yaw.to_bits());
        hasher.write_u64(self.smoothness_roll.to_bits());
        hasher.write_u64(self.max_smoothness.to_bits());
        hasher.write_u64(self.alpha_0_1s.to_bits());
        hasher.write_u8(if self.per_axis { 1 } else { 0 });
        hasher.write_u8(if self.second_pass { 1 } else { 0 });
        hasher.finish()
    }

    fn smooth(&self,
              quats: &TimeQuat, // 方向四元数列表
              duration: f64, // 时间间隔
              stabilization_params: &StabilizationParams, // 稳像相关参数
              keyframes: &KeyframeManager) // 关键帧
              -> TimeQuat { // TODO Result<>? // 返回平滑后的四元数
        if quats.is_empty() || duration <= 0.0 { return quats.clone(); }
        
        // 最大速度
        const MAX_VELOCITY: f64 = 500.0;
        // 弧度转角度
        const RAD_TO_DEG: f64 = 180.0 / std::f64::consts::PI;
        // 采样频率 = 所有数据个数/总时长
        let sample_rate: f64 = quats.len() as f64 / (duration / 1000.0);
        // 每秒的弧度转角度
        let rad_to_deg_per_sec: f64 = sample_rate * RAD_TO_DEG;
        
        // 获取alpha值
        let get_alpha = |time_constant: f64| {
            // 指数映射函数
            // (1.0 / sample_rate)：相邻IMU数据之间的时间间隔（100Hz的IMU数据，则为10ms）
            // time_constant：时间戳，
            1.0 - (-(1.0 / sample_rate) / time_constant).exp()
        };
        // 没有操作 no operation
        let noop = |v| v;

        // 存储拷贝后的方向四元数列表
        let mut _quats_copy = None;
        // 对方向四元数列表进行处理，首尾时间戳之外的所有方向四元数值等于首尾两个方向四元数的值
        let quats = if self.trim_range_only && (stabilization_params.trim_start != 0.0 || stabilization_params.trim_end != 1.0) {
            // 起始时间戳
            let ts_start = ((duration * stabilization_params.trim_start) * 1000.0).round() as i64;
            // 结束时间戳
            let ts_end   = ((duration * stabilization_params.trim_end) * 1000.0).round() as i64;
            // 截取首尾时间戳范围内的四元数值
            if quats.range(ts_start..ts_end).next().is_none() {
                // 若没有四元数，直接返回
                &quats
            } else {
                // 取区间内的第一个方向四元数
                let first_q = quats.range(ts_start..ts_end).next().unwrap().1.clone();
                // 取区内内的最后一个方向四元数
                let last_q = quats.range(ts_start..ts_end).next_back().unwrap().1.clone();
                // 拷贝一份四元数列表
                _quats_copy = Some(quats.clone());
                // 遍历所有方向四元数
                for (ts, q) in _quats_copy.as_mut().unwrap().iter_mut() {
                    // 判断当前四元数的时间戳是否早于起始时间戳
                    if *ts < ts_start {
                        // 若早于，令所有早于该时间戳的四元数值为第一个四元数的值
                        *q = first_q.clone();
                    } else if *ts > ts_end {
                        // 若晚于，令所有晚于该时间戳的四元数值为最后一个四元数的值
                        *q = last_q.clone();
                    }
                }
                _quats_copy.as_ref().unwrap()
            }
        } else {
            &quats
        };

        // 闭包，获取关键参数
        let get_keyframed_param = |typ: &KeyframeType,          // 关键参数类型
                                                                        def: f64,                   // 默认值
                                                                        cb: &dyn Fn(f64) -> f64|    // 回调函数
                                                                        -> BTreeMap<i64, f64> {     // 返回B树
            // 初始化返回容器对象                                      
            let mut ret = BTreeMap::<i64, f64>::new();
            // 判断当前类型是否为关键参数，或者，{视频播放速度是否影响平滑，且，【视频播放速度不为1（倍速播放），或者，视频速度其中一个关键参数】}
            if keyframes.is_keyframed(typ) || (stabilization_params.video_speed_affects_smoothing && (stabilization_params.video_speed != 1.0 || keyframes.is_keyframed(&KeyframeType::VideoSpeed))) {
                ret = quats.iter() // 遍历所有方向四元数
                           .map(|(ts, _)| {  // 闭包函数，只取时间戳
                    // 微秒转毫秒
                    let timestamp_ms = *ts as f64 / 1000.0;
                    // 在IMU数据时间戳处的类型值
                    let mut val = keyframes.value_at_gyro_timestamp(typ, timestamp_ms).unwrap_or(def);
                    // 判断视频播放速度是否影响平滑
                    if stabilization_params.video_speed_affects_smoothing {
                        // 若有影响，
                        // 获取此时IMU数据时间戳处的视频播放速度
                        let vid_speed = keyframes.value_at_gyro_timestamp(&KeyframeType::VideoSpeed, timestamp_ms).unwrap_or(stabilization_params.video_speed);
                        // 判断参数类型是否为时间常量
                        if typ == &KeyframeType::SmoothingParamTimeConstant || typ == &KeyframeType::SmoothingParamTimeConstant2 {
                            // 若是，视频播放速度减1的差除以2，再乘以类型值
                            val *= 1.0 + ((vid_speed - 1.0) / 2.0);
                        } else {
                            // 否则，
                            val *= vid_speed; // 视频播放速度 乘以 类型值
                        }
                    }
                    (*ts, cb(val)) // 调用传入的回调函数（get_alpha(), noop()）
                }).collect();
            }
            ret
        };

        // 六个关键参数
        let alpha_smoothness_per_timestamp = get_keyframed_param(&KeyframeType::SmoothingParamTimeConstant, self.max_smoothness, &get_alpha); // 最大平滑度
        let alpha_0_1s_per_timestamp = get_keyframed_param(&KeyframeType::SmoothingParamTimeConstant2, self.alpha_0_1s, &get_alpha); // 高速下，最大平滑度
        let smoothness_per_timestamp = get_keyframed_param(&KeyframeType::SmoothingParamSmoothness, self.smoothness, &noop); // 平滑度
        let smoothness_pitch_per_timestamp = get_keyframed_param(&KeyframeType::SmoothingParamPitch, self.smoothness_pitch, &noop); // 俯仰角平滑度
        let smoothness_yaw_per_timestamp = get_keyframed_param(&KeyframeType::SmoothingParamYaw, self.smoothness_yaw, &noop); // 航向角平滑度
        let smoothness_roll_per_timestamp = get_keyframed_param(&KeyframeType::SmoothingParamRoll, self.smoothness_roll, &noop); // 横滚角平滑度

        let alpha_smoothness = get_alpha(self.max_smoothness);
        let alpha_0_1s = get_alpha(self.alpha_0_1s);

        // Calculate velocity 计算速度
        // 初始化速度容器
        let mut velocity = BTreeMap::<i64, Vector3<f64>>::new();

        // 获取第一个四元数
        let first_quat = quats.iter().next().unwrap(); // First quat
        // 插入第一个四元数时间戳，速度为0
        velocity.insert(*first_quat.0, Vector3::from_element(0.0));

        // 更新第一个四元数为前一个数据
        let mut prev_quat = *quats.iter().next().unwrap().1; // First quat
        // 【计算相邻IMU数据之间的速度】
        // 遍历剩余方向四元数（跳过第一个四元数）        
        for (timestamp, quat) in quats.iter().skip(1) {
            // 【前一个方向四元数的逆】和【当前方向四元数】作四元数乘法，计算距离差值--旋转欧拉角
            let dist = prev_quat.inverse() * quat;
            // 判断是否按轴分解出三个方向上的欧拉角
            if self.per_axis {
                // 若分解，
                // 四元数转欧拉角, 前后相邻IMU数据之间的旋转欧拉角
                let euler = dist.euler_angles(); // (roll, yaw, pitch)
                // 计算每个IMU数据的速度【来自前后相邻IMU数据方向四元数之间的差值距离 * （IMU采样频率(单位：rad/秒) * （180(deg) / PI(rad)））】
                velocity.insert(*timestamp, Vector3::new( // 绕三个轴的旋转速度（每秒转多少角度）
                    euler.0.abs() * rad_to_deg_per_sec,
                    euler.1.abs() * rad_to_deg_per_sec,
                    euler.2.abs() * rad_to_deg_per_sec
                ));
            } else {
                // dist.angle(): 相邻IMU数据之间的旋转欧拉角
                velocity.insert(*timestamp, Vector3::from_element(dist.angle() * rad_to_deg_per_sec));
            }
            prev_quat = *quat; // 更新
        }

        // Smooth velocity
        // 【速度平滑】
        // 取出第一个速度作为前一个速度值
        let mut prev_velocity = *velocity.iter().next().unwrap().1; // First velocity
        // 遍历剩余IMU数据速度（跳过第一个）
        for (_timestamp, vel) in velocity.iter_mut().skip(1) {
            // 利用比例系数alpha_0_1s，线性插值得到前后IMU数据平滑后的速度
            *vel = prev_velocity * (1.0 - alpha_0_1s) + *vel * alpha_0_1s;
            prev_velocity = *vel; // 更新平滑后的速度
        }
        // 此时的prev_velocity是最后一个IMU数据平滑后的速度
        // 反向遍历，
        for (_timestamp, vel) in velocity.iter_mut().rev().skip(1) {
            *vel = prev_velocity * (1.0 - alpha_0_1s) + *vel * alpha_0_1s;
            prev_velocity = *vel;
        }

        // Normalize velocity 归一化速度
        for (ts, vel) in velocity.iter_mut() {
            // 欧拉角表示旋转时三个轴上的平滑度
            let smoothness_pitch = smoothness_pitch_per_timestamp.get(ts).unwrap_or(&self.smoothness_pitch);
            let smoothness_yaw   = smoothness_yaw_per_timestamp  .get(ts).unwrap_or(&self.smoothness_yaw);
            let smoothness_roll  = smoothness_roll_per_timestamp .get(ts).unwrap_or(&self.smoothness_roll);
            // 每个IMU数据时间戳处，四元数表示旋转时的平滑度
            let smoothness       = smoothness_per_timestamp      .get(ts).unwrap_or(&self.smoothness);

            // Calculate max velocity
            // 计算最大速度
            let mut max_velocity = [MAX_VELOCITY, MAX_VELOCITY, MAX_VELOCITY];
            if self.per_axis {
                max_velocity[0] *= smoothness_pitch;    // 最大速度乘以俯仰角平滑度
                max_velocity[1] *= smoothness_yaw;      // 最大速度乘以偏航角平滑度
                max_velocity[2] *= smoothness_roll;     // 最大速度乘以横滚角平滑度
            } else {
                max_velocity[0] *= smoothness; // 平滑度
            }

            // Doing this to get similar max zoom as without second pass
            // 是否勾选second pass二阶指数平滑
            if self.second_pass {
                max_velocity[0] *= 0.5;
                // 若勾选，最大速度还得除以2
                if self.per_axis {
                    max_velocity[1] *= 0.5;
                    max_velocity[2] *= 0.5;
                }
            }

            // 速度归一化：速度值除以最大速度
            vel[0] /= max_velocity[0];
            if self.per_axis {
                vel[1] /= max_velocity[1];
                vel[2] /= max_velocity[2];
            }
        }

        // 保存中间结果到txt文本中
        // use std::io::prelude::*;
        // let mut quat_raw_file = std::fs::File::create("quat_raw_default_gyroflow_135852.txt").expect("create failed");
        // let mut quat_smoothed_file = std::fs::File::create("quat_smoothed_default_gyroflow_135852.txt").expect("create failed");

        // Plain 3D smoothing with varying alpha
        // Forward pass
        let mut q = *quats.iter().next().unwrap().1;
        let smoothed1: TimeQuat = quats.iter().map(|(ts, // 时间戳
                                                     x)| { // 方向四元数
            // 获取插值比例
            let ratio = velocity[ts]; 
            let alpha_smoothness = alpha_smoothness_per_timestamp.get(ts).unwrap_or(&alpha_smoothness);
            let alpha_0_1s = alpha_0_1s_per_timestamp.get(ts).unwrap_or(&alpha_0_1s);
            if self.per_axis {
                // 线性插值
                let pitch_factor = alpha_smoothness * (1.0 - ratio[0]) + alpha_0_1s * ratio[0];
                let yaw_factor = alpha_smoothness * (1.0 - ratio[1]) + alpha_0_1s * ratio[1];
                let roll_factor = alpha_smoothness * (1.0 - ratio[2]) + alpha_0_1s * ratio[2];

                // 四元数转欧拉角
                let euler_rot = (q.inverse() * x).euler_angles();

                // 欧拉角转四元数
                let quat_rot = Quat64::from_euler_angles(
                    euler_rot.0 * pitch_factor.min(1.0),
                    euler_rot.1 * yaw_factor.min(1.0),
                    euler_rot.2 * roll_factor.min(1.0),
                );
                q *= quat_rot;
            } else {
                // 速度作线性插值
                let val = alpha_smoothness * (1.0 - ratio[0]) + alpha_0_1s * ratio[0];
                q = q.slerp(x, val.min(1.0));
            }
            (*ts, q)
        }).collect();

        // Reverse pass
        let mut q = *smoothed1.iter().next_back().unwrap().1;
        let smoothed2: TimeQuat = smoothed1.into_iter().rev().map(|(ts, // 时间戳
                                                                    x)| { // 方向四元数
            let alpha_smoothness = alpha_smoothness_per_timestamp.get(&ts).unwrap_or(&alpha_smoothness);
            let alpha_0_1s = alpha_0_1s_per_timestamp.get(&ts).unwrap_or(&alpha_0_1s);
            let ratio = velocity[&ts];
            if self.per_axis {
                let pitch_factor = alpha_smoothness * (1.0 - ratio[0]) + alpha_0_1s * ratio[0];
                let yaw_factor = alpha_smoothness * (1.0 - ratio[1]) + alpha_0_1s * ratio[1];
                let roll_factor = alpha_smoothness * (1.0 - ratio[2]) + alpha_0_1s * ratio[2];

                let euler_rot = (q.inverse() * x).euler_angles();

                let quat_rot = Quat64::from_euler_angles(
                    euler_rot.0 * pitch_factor.min(1.0),
                    euler_rot.1 * yaw_factor.min(1.0),
                    euler_rot.2 * roll_factor.min(1.0),
                );
                q *= quat_rot;
            } else {
                let val = alpha_smoothness * (1.0 - ratio[0]) + alpha_0_1s * ratio[0];
                q = q.slerp(&x, val.min(1.0));
            }
            (ts, q)
        }).collect();

        // 是否勾选了 Second smoothing pass 二阶指数平滑单选框
        if !self.second_pass {
            // 若未勾选，直接返回一阶指数平滑后得到的方向四元数
            return smoothed2;
        }

        // Calculate distance
        let mut distance = BTreeMap::<i64, Vector3<f64>>::new();
        let mut max_distance = Vector3::from_element(0.0);
        // 遍历一阶SLERP指数平滑后得到的方向四元数列表
        for (ts, quat) in smoothed2.iter() {
            // 计算平滑前后的方向四元数之间的旋转四元数
            let dist = quats[ts].inverse() * quat;
            if self.per_axis {
                let euler = dist.euler_angles();
                distance.insert(*ts, Vector3::new(
                    euler.0.abs(),
                    euler.1.abs(),
                    euler.2.abs()
                ));
                if euler.0.abs() > max_distance[0] { max_distance[0] = euler.0.abs(); }
                if euler.1.abs() > max_distance[1] { max_distance[1] = euler.1.abs(); }
                if euler.2.abs() > max_distance[2] { max_distance[2] = euler.2.abs(); }
            } else {
                // 添加四元数转旋转角度作为距离
                distance.insert(*ts, Vector3::from_element(dist.angle()));
                // 判断旋转角度是否超过最大距离，若超过了，则更新最大距离值
                if dist.angle() > max_distance[0] { max_distance[0] = dist.angle(); }
            }
        }

        // Normalize distance and discard under 0.5
        // 遍历平滑前后方向四元数的旋转角度列表
        for (_ts, dist) in distance.iter_mut() {
            // 每个距离除以最大距离，完成初始化
            dist[0] /= max_distance[0];
            if dist[0] < 0.5 { dist[0] = 0.0; } // 若距离值小于0.5，则赋为0，直接丢弃
            if self.per_axis {
                dist[1] /= max_distance[1];
                if dist[1] < 0.5 { dist[1] = 0.0; }
                dist[2] /= max_distance[2];
                if dist[2] < 0.5 { dist[2] = 0.0; }
            }
        }

        // Smooth distance 对旋转角度距离作指数平滑
        // 获取第一个旋转角度距离
        let mut prev_dist = *distance.iter().next().unwrap().1;
        // 正向遍历旋转距离列表（跳过第一个距离）
        for (_timestamp, dist) in distance.iter_mut().skip(1) {
            // 指数平滑
            *dist = prev_dist * (1.0 - alpha_0_1s) + *dist * alpha_0_1s;
            prev_dist = *dist;
        }
        // 逆向遍历旋转距离列表（跳过最后一个距离）
        for (_timestamp, dist) in distance.iter_mut().rev().skip(1) {
            *dist = prev_dist * (1.0 - alpha_0_1s) + *dist * alpha_0_1s;
            prev_dist = *dist;
        } // 此时distance中是指数平滑后的距离值

        // Get max distance 计算最大距离
        max_distance = Vector3::from_element(0.0);
        // 遍历指数平滑后的距离列表，计算最大距离
        for (_ts, dist) in distance.iter_mut() {
            if dist[0] > max_distance[0] { max_distance[0] = dist[0]; } // 更新此时的最大距离
            if self.per_axis {
                if dist[1] > max_distance[1] { max_distance[1] = dist[1]; }
                if dist[2] > max_distance[2] { max_distance[2] = dist[2]; }
            }
        }

        // Normalize distance and change range to 0.5 - 1.0
        // 遍历指数平滑后的距离列表，归一化距离值
        for (_ts, dist) in distance.iter_mut() {
            // 每个距离除以新的最大距离
            dist[0] /= max_distance[0];
            // 将距离值控制在0.5 - 1.0之间
            dist[0] = (dist[0] + 1.0) / 2.0;
            if self.per_axis {
                dist[1] /= max_distance[1];
                dist[1] = (dist[1] + 1.0) / 2.0;
                dist[2] /= max_distance[2];
                dist[2] = (dist[2] + 1.0) / 2.0;
            }
        }

        // Plain 3D smoothing with varying alpha
        // Forward pass
        let mut q = *smoothed2.iter().next().unwrap().1;
        let smoothed1: TimeQuat = smoothed2.into_iter().map(|(ts, x)| {
            let alpha_smoothness = alpha_smoothness_per_timestamp.get(&ts).unwrap_or(&alpha_smoothness);
            let alpha_0_1s = alpha_0_1s_per_timestamp.get(&ts).unwrap_or(&alpha_0_1s);
            let vel_ratio = velocity[&ts];
            let dist_ratio = distance[&ts];
            if self.per_axis {
                let pitch_factor = alpha_smoothness * (1.0 - vel_ratio[0] * dist_ratio[0]) + alpha_0_1s * vel_ratio[0] * dist_ratio[0];
                let yaw_factor = alpha_smoothness * (1.0 - vel_ratio[1] * dist_ratio[1]) + alpha_0_1s * vel_ratio[1] * dist_ratio[1];
                let roll_factor = alpha_smoothness * (1.0 - vel_ratio[2] * dist_ratio[2]) + alpha_0_1s * vel_ratio[2] * dist_ratio[2];

                let euler_rot = (q.inverse() * x).euler_angles();

                let quat_rot = Quat64::from_euler_angles(
                    euler_rot.0 * pitch_factor.min(1.0),
                    euler_rot.1 * yaw_factor.min(1.0),
                    euler_rot.2 * roll_factor.min(1.0),
                );
                q *= quat_rot;
            } else {
                let val = alpha_smoothness * (1.0 - vel_ratio[0] * dist_ratio[0]) + alpha_0_1s * vel_ratio[0] * dist_ratio[0];
                q = q.slerp(&x, val.min(1.0));
            }
            (ts, q)
        }).collect();

        // Reverse pass
        let mut q = *smoothed1.iter().next_back().unwrap().1;
        smoothed1.into_iter().rev().map(|(ts, x)| {
            let alpha_smoothness = alpha_smoothness_per_timestamp.get(&ts).unwrap_or(&alpha_smoothness);
            let alpha_0_1s = alpha_0_1s_per_timestamp.get(&ts).unwrap_or(&alpha_0_1s);
            let vel_ratio = velocity[&ts];
            let dist_ratio = distance[&ts];
            if self.per_axis {
                let pitch_factor = alpha_smoothness * (1.0 - vel_ratio[0] * dist_ratio[0]) + alpha_0_1s * vel_ratio[0] * dist_ratio[0];
                let yaw_factor = alpha_smoothness * (1.0 - vel_ratio[1] * dist_ratio[1]) + alpha_0_1s * vel_ratio[1] * dist_ratio[1];
                let roll_factor = alpha_smoothness * (1.0 - vel_ratio[2] * dist_ratio[2]) + alpha_0_1s * vel_ratio[2] * dist_ratio[2];

                let euler_rot = (q.inverse() * x).euler_angles();

                let quat_rot = Quat64::from_euler_angles(
                    euler_rot.0 * pitch_factor.min(1.0),
                    euler_rot.1 * yaw_factor.min(1.0),
                    euler_rot.2 * roll_factor.min(1.0),
                );
                q *= quat_rot;
            } else {
                let val = alpha_smoothness * (1.0 - vel_ratio[0] * dist_ratio[0]) + alpha_0_1s * vel_ratio[0] * dist_ratio[0];
                q = q.slerp(&x, val.min(1.0));
            }
            (ts, q)
        }).collect()
    }
}