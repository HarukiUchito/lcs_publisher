use std::fs::File;
use std::path::Path;
use std::{thread::sleep, time::Duration};

use std::io::{self, BufRead};
use std::process;

use plotters::prelude::*;

use anyhow::{anyhow, Result};

use rclrust::{qos::QoSProfile, rclrust_info, ParameterValue};
use rclrust_msg::geometry_msgs::msg::TwistWithCovarianceStamped as TwistWithCovarianceStamped_;
use rclrust_msg::sensor_msgs::msg::LaserScan as LaserScan_;

mod measurement;
use crate::measurement::parse_line;
use crate::measurement::to_ros_time;
use crate::measurement::Measurement;

fn get_package_share_dir(pkg_name: &str) -> Result<String> {
    let ament_prefix_paths =
        std::env::var("AMENT_PREFIX_PATH").expect("$AMENT_PREFIX_PATH is supposed to be set.");

    for path in ament_prefix_paths.split(':').into_iter() {
        match path.split('/').into_iter().last().clone() {
            Some(name) => {
                if name == pkg_name {
                    let path = path.to_owned() + "/share/" + pkg_name;
                    return Ok(path.to_string());
                }
            }
            None => continue,
        }
    }

    Err(anyhow!("no valid path found"))
}

const NODE_NAME: &str = "lsc_publisher_node";
fn main() -> Result<()> {
    // ros related initialization
    let ctx = rclrust::init()?;
    let node = ctx.create_node(NODE_NAME)?;
    let logger = node.logger();

    let declare_and_get_param = |param, initial_value| {
        match node.declare_parameter(param, &initial_value) {
            Ok(_) => (),
            Err(e) => {
                println!("failed to declare ros parameter {}: {}", param, e);
                process::exit(-1);
            }
        };
        let p = match node.get_parameter(param) {
            Some(p) => p,
            None => {
                println!("ros parameter {} must be defined", param);
                process::exit(-1);
            }
        };
        println!("{:?} is set", p.value.get_value());
        return p.value.get_value().unwrap();
    };
    let base_link_frame_name =
        declare_and_get_param("base_link_frame_name", ParameterValue::string("base_link"))
            .to_string()
            .unwrap();
    let laser_frame_name =
        declare_and_get_param("laser_frame_name", ParameterValue::string("laser"))
            .to_string()
            .unwrap();
    let _play_speed = declare_and_get_param("play_speed", ParameterValue::double(0.1));

    let pub_scan = node.create_publisher::<LaserScan_>("scan", &QoSProfile::default())?;
    let pub_twist =
        node.create_publisher::<TwistWithCovarianceStamped_>("twist", &QoSProfile::default())?;

    // open and read lsc file
    let path = get_package_share_dir(NODE_NAME).unwrap() + "/data/data/corridor.lsc";
    println!("path: {}", path);
    let path = Path::new(&path);
    let file = match File::open(&path) {
        Err(why) => panic!("couldn't open {}: {}", path.display(), why),
        Ok(file) => file,
    };

    let mut cnt = 0;
    let mut measurements: Vec<Measurement> = Vec::new();
    for line in io::BufReader::new(file).lines() {
        match parse_line(line.unwrap()) {
            Some(mut measurement) => {
                measurement.lidar.reset_xy();
                measurement.lidar.interpolate();
                measurements.push(measurement);
            }
            None => {
                println!("failed reading line {} as an measurement", cnt);
            }
        }
        cnt = cnt + 1;
    }

    rclrust_info!(logger, "publish start");
    //return Ok(());
    loop {
        let mstart_time = measurements[0].time;
        let start_time = rclrust::Clock::ros().unwrap().now().unwrap();
        cnt = 0;
        let mut iter = measurements.iter_mut();
        while let Some(m) = iter.next() {
            let md = m.time - mstart_time;
            cnt += 1;
            let mut scan_msg = m.to_ros_laserscan(laser_frame_name.clone());
            let mut twist_msg = m.to_ros_twist(base_link_frame_name.clone());
            loop {
                let ct = rclrust::Clock::ros().unwrap().now().unwrap().nanosecs;
                let td = (ct - start_time.nanosecs) as f64 * 1e-9 * 0.1;
                if td >= md {
                    let rt = ct as f64 * 1e-9;
                    scan_msg.header.stamp = to_ros_time(rt);
                    pub_scan.publish(&scan_msg)?;
                    twist_msg.header.stamp = to_ros_time(rt);
                    pub_twist.publish(&twist_msg)?;
                    println!(
                        "published laser {} on time: {} datan: {}",
                        cnt,
                        ct as f64 * 1e-9,
                        scan_msg.ranges.len()
                    );
                    break;
                }
                sleep(Duration::from_millis(10));
            }
        }
    }

    //Ok(())
    /*
    const num: usize = 149;
    measurements[num].lidar.interpolate_verbose(true);
    let m = &measurements[num];

    let xs = m
        .lidar
        .points
        .iter()
        .map(|p| p.x as f32)
        .collect::<Vec<f32>>();
    let ys = m
        .lidar
        .points
        .iter()
        .map(|p| p.y as f32)
        .collect::<Vec<f32>>();
    let ixs = m
        .lidar
        .interpolated_points
        .iter()
        .map(|p| p.x as f32)
        .collect::<Vec<f32>>();
    let iys = m
        .lidar
        .interpolated_points
        .iter()
        .map(|p| p.y as f32)
        .collect::<Vec<f32>>();
    plot(xs.clone(), ys.clone(), ixs, iys);
    let mut scan_msg = measurements[num].to_ros_laserscan();
    //println!("{:?}", scan_msg.ranges);
    */
}

#[allow(dead_code)]
fn plot(lxs: Vec<f32>, lys: Vec<f32>, pxs: Vec<f32>, pys: Vec<f32>) {
    let root = BitMapBackend::new("plot.png", (1080, 720)).into_drawing_area();

    // 背景を白にする
    root.fill(&WHITE).unwrap();

    let (y_min, y_max) = lys
        .iter()
        .fold((0.0 / 0.0, 0.0 / 0.0), |(m, n), v| (v.min(m), v.max(n)));

    let (x_min, x_max) = lxs
        .iter()
        .fold((0.0 / 0.0, 0.0 / 0.0), |(m, n), v| (v.min(m), v.max(n)));

    let caption = "Sample Plot";
    let font = ("sans-serif", 20);

    let mut chart = ChartBuilder::on(&root)
        .caption(caption, font.into_font()) // キャプションのフォントやサイズ
        .margin(10 as i32) // 上下左右全ての余白
        .x_label_area_size(16 as i32) // x軸ラベル部分の余白
        .y_label_area_size(42 as i32) // y軸ラベル部分の余白
        .build_cartesian_2d(
            // x軸とy軸の数値の範囲を指定する
            x_min..x_max, // x軸の範囲
            y_min..y_max, // y軸の範囲
        )
        .unwrap();

    chart.configure_mesh().draw().unwrap();

    //let line_series = LineSeries::new(lxs.iter().zip(lys.iter()).map(|(x, y)| (*x, *y)), &RED);

    //chart.draw_series(DATA1.iter().map(|point| Circle::new(*point, 5, &BLUE)))?;
    //chart.draw_series(line_series).unwrap();

    let point_series = lxs
        .iter()
        .zip(lys.iter())
        .map(|(x, y)| Circle::new((*x, *y), 4 as i32, &RED));
    chart.draw_series(point_series).unwrap();

    let point_series = pxs
        .iter()
        .zip(pys.iter())
        .map(|(x, y)| Circle::new((*x, *y), 4 as i32, &BLUE));
    chart.draw_series(point_series).unwrap();
}
