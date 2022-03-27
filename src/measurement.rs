use rclrust_msg::geometry_msgs::msg::Twist as Twist_;
use rclrust_msg::geometry_msgs::msg::TwistWithCovariance as TwistWithCovariance_;
use rclrust_msg::geometry_msgs::msg::TwistWithCovarianceStamped as TwistWithCovarianceStamped_;
use rclrust_msg::geometry_msgs::msg::Vector3 as Vector3_;
use rclrust_msg::sensor_msgs::msg::LaserScan as LaserScan_;
use rclrust_msg::std_msgs::msg::Header as Header_;

pub struct Odometry {
    x: f32,
    y: f32,
    theta: f32,
}

#[derive(Clone, Debug, Copy)]
pub struct Point {
    pub x: f32,
    pub y: f32,
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

pub struct LiDAR {
    thetas: Vec<f32>,
    ranges: Vec<f32>,
    pub points: Vec<Point>,
    pub interpolated_points: Vec<Point>,
}

impl LiDAR {
    pub fn reset_xy(self: &mut LiDAR) {
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
    pub fn interpolate(self: &mut LiDAR) {
        self.interpolate_verbose(false);
    }
    pub fn interpolate_verbose(self: &mut LiDAR, verbose: bool) {
        self.interpolated_points.clear();
        let mut iter = self.points.iter().peekable();

        let mut last_th;

        // always add first point
        let mut current_p = match iter.next().cloned() {
            Some(firstpoint) => {
                let fp = firstpoint.clone();
                //self.interpolated_points.push(fp);
                last_th = fp.theta;
                fp
            }
            None => return, // zero points
        };
        let mut last_original_p = current_p;

        let dangle = f32::to_radians(-1.0);
        let mut nangle = self.thetas.first().unwrap() + dangle;
        if verbose {
            println!("first nangle: {}", f32::to_degrees(nangle));
        }
        let mut cnt = 0;
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
            const DIS_THRESHOLD: f32 = 0.05;
            const DEG_THRESHOLD: f32 = 0.5;
            let ddeg = f32::to_degrees(last_th - mid_p.theta).abs();
            if (mc_dist <= DIS_THRESHOLD || mn_dist <= DIS_THRESHOLD) && ddeg >= DEG_THRESHOLD {
                self.interpolated_points.push(mid_p);
                last_th = mid_p.theta;
                current_p = mid_p;
            }

            let lesthan = cnt < 5;
            if verbose && lesthan {
                println!(
                    "\nnangle: {}, cx: {}, cy: {}, cth: {}\n nx: {}, ny: {}, nth: {}",
                    f32::to_degrees(nangle),
                    current_p.x,
                    current_p.y,
                    f32::to_degrees(current_p.theta),
                    next_p.x,
                    next_p.y,
                    f32::to_degrees(next_p.theta)
                );
            }

            last_original_p = next_p;
            nangle += dangle; // update next angle

            if verbose && lesthan {
                println!(
                    "mx: {}, my: {}, mth: {}\n c-dist: {}, n-dist: {}, mth2: {}, ddeg: {}",
                    mid_p.x,
                    mid_p.y,
                    f32::to_degrees(mid_p.theta),
                    mc_dist,
                    mn_dist,
                    f32::to_degrees(mid_p.y.atan2(mid_p.x)),
                    ddeg
                );

                println!("isize: {}", self.interpolated_points.len());
            }
            cnt += 1;
        }
    }
}

pub fn to_ros_time(time: f64) -> rclrust_msg::builtin_interfaces::msg::Time {
    let sec = f64::floor(time);
    let nsec: u32 = ((time - sec) * 1e9) as u32;
    rclrust_msg::builtin_interfaces::msg::Time {
        sec: sec as i32,
        nanosec: nsec as u32,
    }
}

pub struct Measurement {
    pub time: f64,
    pub odometry: Odometry,
    pub lidar: LiDAR,
}

impl Measurement {
    pub fn to_ros_twist(self: &mut Measurement, frame_id: String) -> TwistWithCovarianceStamped_ {
        TwistWithCovarianceStamped_ {
            header: Header_ {
                stamp: to_ros_time(self.time as f64),
                frame_id: frame_id,
            },
            twist: TwistWithCovariance_ {
                twist: Twist_ {
                    linear: Vector3_ {
                        x: self.odometry.x as f64,
                        y: self.odometry.y as f64,
                        z: self.odometry.theta as f64,
                    },
                    angular: Vector3_ {
                        x: 0.0,
                        y: 0.0,
                        z: 0.0,
                    },
                },
                covariance: [0.0; 36],
            },
        }
    }

    pub fn to_ros_laserscan(self: &mut Measurement, frame_id: String) -> LaserScan_ {
        let mut rmin = f32::MAX;
        let mut rmax = f32::MIN;

        let angle_min = self.lidar.interpolated_points.first().unwrap().theta;
        let angle_max = self.lidar.interpolated_points.last().unwrap().theta;
        let angle_inc = f32::to_radians(-1.0);
        let dnum = (f32::abs(angle_max - angle_min) / f32::abs(angle_inc)) as usize;

        //for p in self.lidar.interpolated_points.iter() {
        //println!("{:?}", p);
        //}

        let mut ranges = Vec::new();
        let mut intensities = Vec::new();
        let mut iter = self.lidar.interpolated_points.iter().peekable();
        for i in 0..dnum {
            let theta = angle_min + angle_inc * i as f32;
            match iter.peek() {
                Some(&p) => {
                    //println!("idx: {}, theta: {}", i, f32::to_degrees(theta));
                    //println!("p th: {}, rn: {}", f32::to_degrees(p.theta), p.range);
                    if (p.theta - theta).abs() < 1e-3 {
                        //println!("same");
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

        let t = to_ros_time(self.time as f64);
        //println!("time {}.{}", t.sec, t.nanosec);
        LaserScan_ {
            header: Header_ {
                frame_id: frame_id,
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

pub fn parse_line(line: String) -> Option<Measurement> {
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
