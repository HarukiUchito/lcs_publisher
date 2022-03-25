use std::fs::File;
use std::iter::empty;
use std::path::Path;
use std::{thread::sleep, time::Duration};

use std::io::{self, BufRead};

use plotters::prelude::*;

use anyhow::Result;
use rclrust::Time;
use rclrust::{qos::QoSProfile, rclrust_info};
use rclrust_msg::sensor_msgs::msg::LaserScan as LaserScan_;
use rclrust_msg::std_msgs::msg::Header as Header_;
use rclrust_msg::std_msgs::msg::String as String_;

struct Odometry {
    x: f32,
    y: f32,
    theta: f32,
}

#[derive(Clone, Debug, Copy)]
struct Point {
    x: f32,
    y: f32,
    theta: f32,
    range: f32,
}

fn distance(p1: Point, p2: Point) -> f32 {
    let dx = p1.x - p2.x;
    let dy = p1.y - p2.y;
    f32::sqrt(dx * dx + dy * dy)
}

fn angular_mid(p1: Point, p2: Point, theta: f32) -> Point {
    let a = (p1.y - p2.y) / (p1.x - p2.x);
    let b = p1.y - a * p1.x;
    let tant = f32::tan(theta);
    let mx = b / (tant - a);
    let my = tant * mx;
    Point {
        x: mx,
        y: my,
        theta: theta,
        range: f32::sqrt(mx * mx + my * my),
    }
}

struct LiDAR {
    thetas: Vec<f32>,
    ranges: Vec<f32>,
    points: Vec<Point>,
    interpolated_points: Vec<Point>,
}

impl LiDAR {
    fn reset_xy(self: &mut LiDAR) {
        self.points.clear();
        for i in 0..self.thetas.len() {
            let th = self.thetas[i];
            let rn = self.ranges[i];
            if rn < 0.1 || rn > 6.0 {
                continue;
            }
            let x = rn * f32::cos(th);
            let y = rn * f32::sin(th);
            self.points.push(Point {
                x: x,
                y: y,
                theta: th,
                range: rn,
            });
            //println!("th: {}, rn: {}, x: {}, y: {}", th, rn, x, y);
        }
    }
    fn interpolate(self: &mut LiDAR) {
        self.interpolated_points.clear();
        let mut iter = self.points.iter().peekable();

        // always add first point
        let mut current_p = match iter.next().cloned() {
            Some(firstpoint) => {
                let fp = firstpoint.clone();
                self.interpolated_points.push(fp);
                fp
            }
            None => return, // zero points
        };
        let mut last_original_p = current_p;

        let dangle = f32::to_radians(-1.0);
        let mut nangle = self.thetas.first().unwrap() + dangle;
        loop {
            let mut next_p: Point;
            let next_p = loop {
                next_p = match iter.peek().cloned() {
                    Some(np) => np.clone(),
                    None => break None, // if no more next point
                };
                if next_p.theta < nangle {
                    break Some(next_p);
                }
                iter.next();
            };
            let next_p = match next_p {
                Some(np) => np,
                None => break,
            };
            let mid_p = angular_mid(current_p, next_p, nangle);
            let mc_dist = distance(mid_p, last_original_p);
            let mn_dist = distance(mid_p, next_p);
            const D_THRESHOLD: f32 = 0.05;
            if mc_dist <= D_THRESHOLD || mn_dist <= D_THRESHOLD {
                self.interpolated_points.push(mid_p);
            }

            /*
            println!(
                "\ncx: {}, cy: {}, cth: {}\n nx: {}, ny: {}, nth: {}",
                current_p.x,
                current_p.y,
                f32::to_degrees(current_p.theta),
                next_p.x,
                next_p.y,
                f32::to_degrees(next_p.theta)
            );
            */

            last_original_p = next_p;
            current_p = mid_p;
            nangle += dangle; // update next angle

            /* println!(
                "mx: {}, my: {}, mth: {}\n c-dist: {}, n-dist: {}, mth2: {}",
                mid_p.x,
                mid_p.y,
                f32::to_degrees(mid_p.theta),
                mc_dist,
                mn_dist,
                f32::to_degrees(mid_p.y.atan2(mid_p.x))
            ); */

            // println!("isize: {}", self.interpolated_points.len());
        }
    }
}

fn toRosTime(time: f64) -> rclrust_msg::builtin_interfaces::msg::Time {
    let sec = f64::floor(time);
    let nsec: u32 = ((time - sec) * 1e9) as u32;
    rclrust_msg::builtin_interfaces::msg::Time {
        sec: sec as i32,
        nanosec: nsec as u32,
    }
}

struct Measurement {
    time: f64,
    odometry: Odometry,
    lidar: LiDAR,
}

impl Measurement {
    fn to_ros_laserscan(self: &mut Measurement) -> LaserScan_ {
        let mut rmin = f32::MAX;
        let mut rmax = f32::MIN;

        let angle_min = self.lidar.interpolated_points.first().unwrap().theta;
        let angle_max = self.lidar.interpolated_points.last().unwrap().theta;
        let angle_inc = f32::to_radians(-1.0);
        let dnum = (f32::abs(angle_max - angle_min) / f32::abs(angle_inc)) as usize;

        let mut ranges = Vec::new();
        let mut intensities = Vec::new();
        let mut iter = self.lidar.interpolated_points.iter().peekable();
        for i in 0..dnum {
            let theta = angle_min + angle_inc * i as f32;
            match iter.peek() {
                Some(&p) => {
                    if p.theta == theta {
                        ranges.push(p.range);
                        iter.next();
                    } else {
                        ranges.push(f32::MAX);
                    }
                    intensities.push(0.0);
                    rmin = rmin.min(p.range);
                    rmax = rmax.max(p.range);
                }
                None => break,
            }
        }
        assert!(
            ranges.len() == dnum,
            "rangelen {} dnum {}",
            ranges.len(),
            dnum
        );

        let t = toRosTime(self.time as f64);
        println!("time {}.{}", t.sec, t.nanosec);
        LaserScan_ {
            header: Header_ {
                frame_id: "laser".to_string(),
                stamp: t,
            },
            angle_min: angle_min,
            angle_max: angle_max,
            angle_increment: angle_inc,
            range_min: rmin,
            range_max: rmax,
            scan_time: 0.0,
            time_increment: 0.0,
            ranges: ranges,
            intensities: intensities,
        }
    }
}

fn parse_line(line: String) -> Option<Measurement> {
    let mut tokens = line.split_ascii_whitespace().collect::<Vec<&str>>();

    let tlen = tokens.len() as i32;
    //println!("token num: {}", tlen);
    if tlen < 5 {
        return None;
    }

    let ltype = tokens[0];
    if ltype != "LASERSCAN" {
        return None;
    }
    // let id = tokens[1].parse::<i32>().unwrap();
    let sec = tokens[2].parse::<i32>().unwrap();
    let nsec = tokens[3].parse::<i32>().unwrap();
    let fsec = sec as f64 + (nsec as f64 * 1e-9);

    let pnum = tokens[4].parse::<i32>().unwrap();
    let rest = tlen - 2 * pnum;
    /*println!(
        "sec: {}, nsec: {}, nsec9: {}, time: {}",
        sec,
        nsec,
        nsec as f32 * 1e-9,
        fsec
    );*/
    if rest < 8 {
        return None;
    }

    let idx = tokens.len() - 5;
    //println!("from: {}", idx);
    tokens.drain(idx..(idx + 5));
    //println!("last token: {}", tokens.last().unwrap());
    tokens.drain(0..5); // remove until pnum
                        //println!("first token: {}", tokens.first().unwrap());

    let thetas = tokens
        .iter()
        .step_by(2)
        .map(|s| f32::to_radians(s.parse::<f32>().unwrap()))
        .collect::<Vec<f32>>();
    // let theta_ds = thetas.windows(2).map(|x| x[0] - x[1]).collect::<Vec<f32>>();

    tokens.drain(0..1); // remove first theta

    let ranges = tokens
        .iter()
        .step_by(2)
        .map(|s| s.parse::<f32>().unwrap())
        .collect::<Vec<f32>>();

    Some(Measurement {
        time: fsec,
        odometry: Odometry {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
        },
        lidar: LiDAR {
            thetas: thetas,
            ranges: ranges,
            points: Vec::new(),
            interpolated_points: Vec::new(),
        },
    })
}

/*
   println!(
       "mind {}, maxd {}",
       theta_ds.iter().cloned().fold(-1. / 0. /* -inf */, f32::max),
       theta_ds.iter().copied().fold(f32::NEG_INFINITY, f32::max)
   );
*/

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
            x_min..y_max, // x軸の範囲
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

fn main() -> Result<()> {
    let ctx = rclrust::init()?;
    let node = ctx.create_node("lsc_publisher_node")?;
    let logger = node.logger();
    let publisher = node.create_publisher::<LaserScan_>("scan", &QoSProfile::default())?;

    let path = Path::new("data/hall.lsc");
    let display = path.display();

    let mut file = match File::open(&path) {
        Err(why) => panic!("couldn't open {}: {}", display, why),
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
                /*
                if cnt < 1 {
                    let xs = measurement
                        .lidar
                        .points
                        .iter()
                        .map(|p| p.x as f32)
                        .collect::<Vec<f32>>();
                    let ys = measurement
                        .lidar
                        .points
                        .iter()
                        .map(|p| p.y as f32)
                        .collect::<Vec<f32>>();
                    let ixs = measurement
                        .lidar
                        .interpolated_points
                        .iter()
                        .map(|p| p.x as f32)
                        .collect::<Vec<f32>>();
                    let iys = measurement
                        .lidar
                        .interpolated_points
                        .iter()
                        .map(|p| p.y as f32)
                        .collect::<Vec<f32>>();
                    plot(xs.clone(), ys.clone(), ixs, iys);
                }
                    */
            }
            None => {
                println!("failed reading line {} as an measurement", cnt);
            }
        }
        cnt = cnt + 1;
    }

    let mstart_time = measurements[0].time;
    let start_time = rclrust::Clock::ros().unwrap().now().unwrap();
    cnt = 0;
    let mut iter = measurements.iter_mut();
    while let Some(m) = iter.next() {
        let md = m.time - mstart_time;
        cnt += 1;
        let mut scan_msg = m.to_ros_laserscan();
        loop {
            let ct = rclrust::Clock::ros().unwrap().now().unwrap().nanosecs;
            let td = (ct - start_time.nanosecs) as f64 * 1e-9;
            if td >= md {
                scan_msg.header.stamp = toRosTime(ct as f64 * 1e-9);
                publisher.publish(&scan_msg)?;
                //rclrust_info!(logger,
                println!("published laser {} on time: {}", cnt, ct as f64 * 1e-9);
                break;
            }
            sleep(Duration::from_millis(10));
        }
    }

    Ok(())
}
