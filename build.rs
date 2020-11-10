use std::fs::File;
use std::path::Path;
use std::io::prelude::*;
use std::f32::consts::PI;

const TABLE_N: usize = 1_000;

fn construct_table(fname: String, xvals: [f32; TABLE_N], tr: fn(f32) -> f32) {
    let path = Path::new("src").join(fname);
    let display = path.display();

    let mut file = match File::create(&path) {
        Err(why) => panic!("failed to write to {}: {}", display, why),
        Ok(file) => file,
    };

    match file.write_all("[\n".as_bytes()) {
        Err(why) => panic!("failed to write to {}: {}", display, why),
        Ok(_) => (),
    }

    for val in xvals.iter() {
        let s = format!("  {:.34},\n", tr(*val));
        match file.write_all(s.as_bytes()) {
            Err(why) => panic!("failed to write to {}: {}", display, why),
            Ok(_) => (),
        }
    }

    match file.write_all("]\n".as_bytes()) {
        Err(why) => panic!("failed to write to {}: {}", display, why),
        Ok(_) => (),
    }
}

fn main() {
    let sin_fname = String::from("sin_table.txt");
    let sin_delta: f32 = PI / 2.0 / (TABLE_N-1) as f32;
    let mut sin_xvals = [0f32; TABLE_N];
    for i in 0..TABLE_N {
        sin_xvals[i] = sin_delta * i as f32;
    }
    construct_table(sin_fname, sin_xvals, f32::sin);

    let atan_fname = String::from("atan_table.txt");
    let atan_delta: f32 = 1. / (TABLE_N-1) as f32;
    let mut atan_xvals = [0f32; TABLE_N];
    for i in 0..TABLE_N {
        atan_xvals[i] = atan_delta * i as f32;
    }
    construct_table(atan_fname, atan_xvals, f32::atan);
}
