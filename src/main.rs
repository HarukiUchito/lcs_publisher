use std::fs::File;
use std::iter::empty;
use std::path::Path;
use std::{thread::sleep, time::Duration};

use std::io::{self, BufRead};

use plotters::prelude::*;

use anyhow::Result;
use rclrust::{qos::QoSProfile, rclrust_info};
use rclrust_msg::sensor_msgs::msg::LaserScan as LaserScan_;
use rclrust_msg::std_msgs::msg::String as String_;

struct Odometry {
    x: f64,
    y: f64,
    theta: f64,
}

#[derive(Clone, Debug, Copy)]
struct Point {
    x: f64,
    y: f64,
    theta: f64,
}

fn distance(p1: Point, p2: Point) -> f64 {
    let dx = p1.x - p2.x;
    let dy = p1.y - p2.y;
    f64::sqrt(dx * dx + dy * dy)
}

fn angular_mid(p1: Point, p2: Point, theta: f64) -> Point {
    let a = (p1.y - p2.y) / (p1.x - p2.x);
    let b = p1.y - a * p1.x;
    let tant = f64::tan(theta);
    let mx = b / (tant - a);
    let my = tant * mx;
    Point {
        x: mx,
        y: my,
        theta: theta,
    }
}

struct LiDAR {
    thetas: Vec<f64>,
    ranges: Vec<f64>,
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
            let x = rn * f64::cos(th);
            let y = rn * f64::sin(th);
            self.points.push(Point {
                x: x,
                y: y,
                theta: th,
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

        let dangle = f64::to_radians(-1.0);
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
            const D_THRESHOLD: f64 = 0.05;
            if mc_dist <= D_THRESHOLD || mn_dist <= D_THRESHOLD {
                self.interpolated_points.push(mid_p);
            }

            /* println!(
                "\ncx: {}, cy: {}, cth: {}\n nx: {}, ny: {}, nth: {}",
                current_p.x,
                current_p.y,
                f64::to_degrees(current_p.theta),
                next_p.x,
                next_p.y,
                f64::to_degrees(next_p.theta)
            ); */

            last_original_p = next_p;
            current_p = mid_p;
            nangle += dangle; // update next angle

            /* println!(
                "mx: {}, my: {}, mth: {}\n c-dist: {}, n-dist: {}, mth2: {}",
                mid_p.x,
                mid_p.y,
                f64::to_degrees(mid_p.theta),
                mc_dist,
                mn_dist,
                f64::to_degrees(mid_p.y.atan2(mid_p.x))
            ); */

            // println!("isize: {}", self.interpolated_points.len());
        }
    }
}

struct Measurement {
    time: f64,
    odometry: Odometry,
    lidar: LiDAR,
}

fn parse_line(line: String) -> Option<Measurement> {
    let mut tokens = line.split_ascii_whitespace().collect::<Vec<&str>>();

    let tlen = tokens.len() as i32;
    println!("token num: {}", tlen);
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
    let fsec = sec as f64 + ((nsec as f64) * 1.0e-9);
    print!("time {:?}", fsec);

    let pnum = tokens[4].parse::<i32>().unwrap();
    let rest = tlen - 2 * pnum;
    println!(", pnum: {}, rest: {}", pnum, rest);
    if rest < 8 {
        return None;
    }

    let idx = tokens.len() - 5;
    println!("from: {}", idx);
    tokens.drain(idx..(idx + 5));
    println!("last token: {}", tokens.last().unwrap());
    tokens.drain(0..5); // remove until pnum
    println!("first token: {}", tokens.first().unwrap());

    let thetas = tokens
        .iter()
        .step_by(2)
        .map(|s| f64::to_radians(s.parse::<f64>().unwrap()))
        .collect::<Vec<f64>>();
    // let theta_ds = thetas.windows(2).map(|x| x[0] - x[1]).collect::<Vec<f64>>();

    tokens.drain(0..1); // remove first theta

    let ranges = tokens
        .iter()
        .step_by(2)
        .map(|s| s.parse::<f64>().unwrap())
        .collect::<Vec<f64>>();

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
       theta_ds.iter().cloned().fold(-1. / 0. /* -inf */, f64::max),
       theta_ds.iter().copied().fold(f64::NEG_INFINITY, f64::max)
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
    let publisher = node.create_publisher::<String_>("message", &QoSProfile::default())?;

    let path = Path::new("data/hall.lsc");
    let display = path.display();

    let mut file = match File::open(&path) {
        Err(why) => panic!("couldn't open {}: {}", display, why),
        Ok(file) => file,
    };

    let mut cnt = 0;
    for line in io::BufReader::new(file).lines() {
        if cnt < 1 {
            let mut measurement = parse_line(line.unwrap());
            match measurement {
                Some(mut measurement) => {
                    measurement.lidar.reset_xy();
                    measurement.lidar.interpolate();
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
                None => {}
            }
        }
        cnt = cnt + 1;
    }

    for count in 0..2 {
        let mut laserMsg: LaserScan_;

        publisher.publish(&String_ {
            data: format!("hello {}", count),
        })?;
        rclrust_info!(logger, "hello {}", count);
        sleep(Duration::from_millis(100));
    }

    Ok(())
}
