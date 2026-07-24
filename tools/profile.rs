use std::process::ExitCode;

fn usage() -> String {
    let planners = nanoplan::profile::planner_ids()
        .map(|id| format!("    {id}"))
        .collect::<Vec<_>>()
        .join("\n");
    let downloaded_tracks = nanoplan::profile::downloaded_track_ids()
        .map(|id| format!("    {id}"))
        .collect::<Vec<_>>()
        .join("\n");
    format!(
        "\
Usage: cargo run --release --bin profile -- [OPTIONS]

Options:
  --planner NAME          Planner key (default: lattice)
  --track NAME            Track id (default: small)
  --laps NUMBER           Number of laps, including fractions (default: 1)
  --start-fraction NUMBER Fraction along the route at which to start (default: 0)
  --transverse METERS     Initial offset left of the centerline (default: 0)
  --yaw-offset RADIANS    Initial yaw offset from the centerline (default: 0)
  --speed MPS             Initial speed (default: 0)
  -h, --help              Show this help

Planner ids:
{planners}

Track ids:
    large
    small
{downloaded_tracks}

Example:
  cargo run --release --bin profile -- --planner lattice --track small --laps 1 \\
    --start-fraction 0.25 --transverse 1 --yaw-offset 0.1 --speed 20
"
    )
}

fn value(args: &mut impl Iterator<Item = String>, flag: &str) -> Result<String, String> {
    args.next()
        .ok_or_else(|| format!("{flag} requires a value"))
}

fn number(args: &mut impl Iterator<Item = String>, flag: &str) -> Result<f64, String> {
    let raw = value(args, flag)?;
    raw.parse()
        .map_err(|error| format!("invalid {flag} value {raw:?}: {error}"))
}

fn main() -> ExitCode {
    match run() {
        Ok(true) => ExitCode::SUCCESS,
        Ok(false) => ExitCode::FAILURE,
        Err(error) => {
            eprintln!("error: {error}\n\n{}", usage());
            ExitCode::FAILURE
        }
    }
}

fn run() -> Result<bool, String> {
    let mut planner = "lattice".to_owned();
    let mut track = "small".to_owned();
    let mut laps = 1.0;
    let mut initial = nanoplan::profile::InitialState::default();
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--planner" => planner = value(&mut args, "--planner")?,
            "--track" => track = value(&mut args, "--track")?,
            "--laps" => {
                let raw = value(&mut args, "--laps")?;
                laps = raw
                    .parse()
                    .map_err(|error| format!("invalid --laps value {raw:?}: {error}"))?;
            }
            "--start-fraction" => {
                initial.route_fraction = number(&mut args, "--start-fraction")?;
            }
            "--transverse" => initial.transverse = number(&mut args, "--transverse")?,
            "--yaw-offset" => initial.yaw_offset = number(&mut args, "--yaw-offset")?,
            "--speed" => initial.speed = number(&mut args, "--speed")?,
            "-h" | "--help" => {
                print!("{}", usage());
                return Ok(true);
            }
            _ => return Err(format!("unknown argument {arg:?}")),
        }
    }

    let profile = nanoplan::profile::run_from(&planner, &track, laps, initial)?;
    println!(
        "{} on {}: {:.3}/{:.3} laps, {:.1}s simulated ({} ticks), {:.3}ms wall, {} contacts",
        profile.planner,
        profile.track,
        profile.completed_laps,
        profile.requested_laps,
        profile.simulated_seconds,
        profile.ticks,
        profile.wall_ms,
        profile.collision_count,
    );
    for seam in &profile.seams {
        println!(
            "{:<28} calls {:>5}  mean {:>8.3}ms  max {:>8.3}ms  clocks {:>8.1}/{:>6} total {}",
            seam.name,
            seam.calls,
            seam.mean_ms,
            seam.max_ms,
            seam.mean_clocks,
            seam.max_clocks,
            seam.total_clocks,
        );
    }

    if !profile.completed {
        eprintln!("run did not reach the requested lap fraction");
    }
    if profile.collision_count != 0 {
        eprintln!("run contacted a road barrier or opponent");
    }
    Ok(profile.completed && profile.collision_count == 0)
}
