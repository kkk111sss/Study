// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright © 2022 Adrian <adrian.eddy at gmail>

use crate::{ stabilization::KernelParams, lens_profile::LensProfile };

#[derive(Default, Clone)]
pub struct GoProHyperview { }

impl GoProHyperview {
    /// `uv` range: (0,0)...(width, height)
    /// From hyperview to wide
    pub fn undistort_point(&self, mut uv: (f32, f32), params: &KernelParams) -> Option<(f32, f32)> {
        let out_c2 = (params.output_width as f32, params.output_height as f32);
        uv = ((uv.0 / out_c2.0) - 0.5,
              (uv.1 / out_c2.1) - 0.5);

        uv.0 *= 1.0 - 0.64 * uv.0.abs();
        uv.0 *= 1.0101 * (1.0 - 0.0294118 * uv.0.abs());
        uv.1 *= 1.0101 * (1.0 - 0.0200000 * uv.1.abs());

        Some(((uv.0 + 0.5) * out_c2.0,
              (uv.1 + 0.5) * out_c2.1))
    }

    /// `uv` range: (0,0)...(width, height)
    /// From wide to hyperview
    pub fn distort_point(&self, mut x: f32, mut y: f32, _z: f32, params: &KernelParams) -> (f32, f32) {
        let size = (params.width as f32, params.height as f32);
        x = (x / size.0) - 0.5;
        y = (y / size.1) - 0.5;

        let xs = if x < 0.0 { -1.0 } else { 1.0 };
        let ys = if y < 0.0 { -1.0 } else { 1.0 };

        y = ys * (-25.0 * ((1.0 - 0.0792 * y.abs()).sqrt() - 1.0));
        x = xs * (-25.0 * (0.824621 * (0.68 - 0.0792 * x.abs()).sqrt() - 0.68));
        x = xs * (-0.78125 * ((1.0 - 2.56 * x.abs()).sqrt() - 1.0));

        ((x + 0.5) * size.0,
         (y + 0.5) * size.1)
    }
    pub fn adjust_lens_profile(&self, profile: &mut LensProfile) {
        let aspect = (profile.calib_dimension.w as f64 / profile.calib_dimension.h as f64 * 100.0) as usize;
        if aspect == 114 { // It's 8:7
            profile.calib_dimension.w = (profile.calib_dimension.w as f64 * 1.55555555555).round() as usize;
        }
        profile.lens_model = "Hyperview".into();
    }

    pub fn id()   -> &'static str { "gopro_hyperview" }
    pub fn name() -> &'static str { "GoPro Hyperview" }

    pub fn opencl_functions(&self) -> &'static str {
        r#"
        float2 digital_undistort_point(float2 uv, __global KernelParams *params) {
            float2 out_c2 = (float2)(params->output_width, params->output_height);
            uv = (uv / out_c2) - 0.5f;

            uv.x *= 1.0f - 0.64f * fabs(uv.x);
            uv.x *= 1.0101f * (1.0f - 0.0294118f * fabs(uv.x));
            uv.y *= 1.0101f * (1.0f - 0.0200000f * fabs(uv.y));

            uv = (uv + 0.5f) * out_c2;

            return uv;
        }
        float2 digital_distort_point(float2 uv, __global KernelParams *params) {
            float2 size = (float2)(params->width, params->height);
            uv = (uv / size) - 0.5f;

            float xs = uv.x < 0.0f? -1.0f : 1.0f;
            float ys = uv.y < 0.0f? -1.0f : 1.0f;

            uv.y = ys * (-25.0f * (sqrt(1.0f - 0.0792f * fabs(uv.y)) - 1.0f));
            uv.x = xs * (-25.0f * (0.824621f * sqrt(0.68f - 0.0792f * fabs(uv.x)) - 0.68f));
            uv.x = xs * (-0.78125f * (sqrt(1.0f - 2.56f * fabs(uv.x)) - 1.0f));

            uv = (uv + 0.5f) * size;

            return uv;
        }"#
    }
    pub fn wgsl_functions(&self) -> &'static str {
        r#"
        fn digital_undistort_point(_uv: vec2<f32>) -> vec2<f32> {
            let out_c2 = vec2<f32>(f32(params.output_width), f32(params.output_height));
            var uv = _uv;
            uv = (uv / out_c2) - 0.5;

            uv.x = uv.x * (1.0 - 0.64 * abs(uv.x));
            uv.x = uv.x * (1.0101 * (1.0 - 0.0294118 * abs(uv.x)));
            uv.y = uv.y * (1.0101 * (1.0 - 0.0200000 * abs(uv.y)));

            uv = (uv + 0.5) * out_c2;

            return uv;
        }
        fn digital_distort_point(_uv: vec2<f32>) -> vec2<f32> {
            let size = vec2<f32>(f32(params.width), f32(params.height));
            var uv = _uv;
            uv = (uv / size) - 0.5;

            let xs = uv.x / max(0.000001, abs(uv.x));
            let ys = uv.y / max(0.000001, abs(uv.y));

            uv.y = ys * (-25.0 * (sqrt(1.0 - 0.0792 * abs(uv.y)) - 1.0));
            uv.x = xs * (-25.0 * (0.824621 * sqrt(0.68 - 0.0792 * abs(uv.x)) - 0.68));
            uv.x = xs * (-0.78125 * (sqrt(1.0 - 2.56 * abs(uv.x)) - 1.0));

            uv = (uv + 0.5) * size;

            return uv;
        }"#
    }
}
