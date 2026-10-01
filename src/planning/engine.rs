use web_time::Instant;

use crate::planning::{ComputeBudget, Context, Diagnostics, DiagnosticsData, Latency, Planner, PlannerKind, Span};
use crate::simulation::{Control, State};
use crate::track::{Road, RoadView};

#[cfg_attr(target_family = "wasm", derive(serde::Deserialize, serde::Serialize))]
pub(crate) struct PlanRequest {
    pub(crate) tick: u64,
    pub(crate) ego: State,
    pub(crate) road: RoadView,
    pub(crate) actors: Vec<State>,
    pub(crate) horizon: usize,
    pub(crate) compute_budget: ComputeBudget,
    pub(crate) diagnostics_enabled: bool,
}

#[cfg_attr(target_family = "wasm", derive(serde::Deserialize, serde::Serialize))]
pub(crate) struct PlanResult {
    pub(crate) tick: u64,
    pub(crate) controls: Vec<Control>,
    pub(crate) diagnostics: DiagnosticsData,
    pub(crate) latency: Vec<Span>,
    pub(crate) elapsed_ms: f64,
}

fn run(planner: &mut dyn Planner, road: &Road, request: PlanRequest) -> PlanResult {
    let latency = Latency::default();
    let diagnostics = Diagnostics::default();
    let ctx = Context::new(
        road,
        &request.actors,
        request.horizon,
        request.compute_budget,
        Some(&latency),
        request.diagnostics_enabled.then_some(&diagnostics),
    );
    let start = Instant::now();
    let controls = latency.time("planner.total", || planner.plan(request.ego, &ctx));
    PlanResult {
        tick: request.tick,
        controls,
        diagnostics: diagnostics.take(),
        latency: latency.take(),
        elapsed_ms: start.elapsed().as_secs_f64() * 1e3,
    }
}

#[cfg_attr(target_family = "wasm", derive(serde::Deserialize, serde::Serialize))]
struct WorkerRequest {
    generation: u64,
    kind: PlannerKind,
    plan: PlanRequest,
}

#[cfg_attr(target_family = "wasm", derive(serde::Deserialize, serde::Serialize))]
struct WorkerResult {
    generation: u64,
    plan: PlanResult,
}

struct PreparedPlanner {
    road: Road,
    kind: PlannerKind,
    generation: u64,
    planner: Box<dyn Planner>,
}

impl PreparedPlanner {
    fn run(&mut self, request: WorkerRequest) -> WorkerResult {
        if self.kind != request.kind || self.generation != request.generation {
            self.planner = request.kind.build();
            self.kind = request.kind;
            self.generation = request.generation;
        }
        let road = self.road.with_view(request.plan.road.clone());
        WorkerResult {
            generation: request.generation,
            plan: run(&mut *self.planner, &road, request.plan),
        }
    }
}

#[cfg(not(target_family = "wasm"))]
mod platform {
    use super::*;
    use std::sync::mpsc::{Receiver, Sender, channel};

    pub(crate) struct PlannerEngine {
        requests: Sender<WorkerRequest>,
        results: Receiver<WorkerResult>,
        pending_since: Option<Instant>,
        kind: PlannerKind,
        generation: u64,
    }

    impl PlannerEngine {
        pub(crate) fn new(kind: PlannerKind, _track_index: usize, road: Road, _initial_speed: f64) -> Self {
            Self::spawn(kind, kind.build(), road)
        }

        #[cfg(test)]
        pub(crate) fn with_planner(planner: Box<dyn Planner>, road: Road) -> Self {
            Self::spawn(PlannerKind::Straight, planner, road)
        }

        fn spawn(kind: PlannerKind, planner: Box<dyn Planner>, road: Road) -> Self {
            road.prepare();
            let (requests, request_rx) = channel::<WorkerRequest>();
            let (result_tx, results) = channel();
            std::thread::Builder::new()
                .name("nanoplan-planner".into())
                .spawn(move || {
                    let mut prepared = PreparedPlanner {
                        road,
                        kind,
                        generation: 0,
                        planner,
                    };
                    while let Ok(request) = request_rx.recv() {
                        if result_tx.send(prepared.run(request)).is_err() {
                            break;
                        }
                    }
                })
                .expect("planner worker thread should start");
            Self {
                requests,
                results,
                pending_since: None,
                kind,
                generation: 0,
            }
        }

        pub(crate) fn is_prepared(&mut self) -> bool {
            true
        }

        pub(crate) fn reset(&mut self, kind: PlannerKind) {
            self.kind = kind;
            self.generation += 1;
            self.pending_since = None;
        }

        pub(crate) fn submit(&mut self, plan: PlanRequest) -> bool {
            if self.pending_since.is_some() {
                return false;
            }
            if self
                .requests
                .send(WorkerRequest {
                    generation: self.generation,
                    kind: self.kind,
                    plan,
                })
                .is_err()
            {
                return false;
            }
            self.pending_since = Some(Instant::now());
            true
        }

        pub(crate) fn poll(&mut self) -> Option<PlanResult> {
            while let Ok(result) = self.results.try_recv() {
                if result.generation == self.generation {
                    self.pending_since = None;
                    return Some(result.plan);
                }
            }
            None
        }

        pub(crate) fn is_slow(&self, planning_period_s: f64) -> bool {
            self.pending_since
                .is_some_and(|start| start.elapsed().as_secs_f64() > planning_period_s)
        }

        pub(crate) fn wait(&mut self) -> PlanResult {
            loop {
                let result = self.results.recv().expect("planner worker should return a result");
                if result.generation == self.generation {
                    self.pending_since = None;
                    return result.plan;
                }
            }
        }
    }
}

#[cfg(target_family = "wasm")]
mod platform {
    use super::*;
    use gloo_worker::{HandlerId, Registrable, Spawnable, Worker, WorkerBridge, WorkerScope};
    use std::cell::RefCell;
    use std::collections::VecDeque;
    use std::rc::Rc;

    #[derive(serde::Deserialize, serde::Serialize)]
    enum Input {
        Prepare { track: usize, dt: f64, initial_speed: f64 },
        Plan(WorkerRequest),
    }

    #[derive(serde::Deserialize, serde::Serialize)]
    enum Output {
        Ready,
        Plan(WorkerResult),
    }

    struct PlannerWorker {
        prepared: Option<PreparedPlanner>,
    }

    impl Worker for PlannerWorker {
        type Message = ();
        type Input = Input;
        type Output = Output;
        fn create(_scope: &WorkerScope<Self>) -> Self {
            Self { prepared: None }
        }
        fn update(&mut self, _scope: &WorkerScope<Self>, _message: Self::Message) {}
        fn received(&mut self, scope: &WorkerScope<Self>, request: Self::Input, id: HandlerId) {
            match request {
                Input::Prepare {
                    track,
                    dt,
                    initial_speed,
                } => {
                    let track = crate::track::Track::from_catalog(track);
                    let mut road = track.prepare(initial_speed, dt).planning.clone();
                    road.dt = dt;
                    self.prepared = Some(PreparedPlanner {
                        road,
                        kind: PlannerKind::Straight,
                        generation: 0,
                        planner: PlannerKind::Straight.build(),
                    });
                    scope.respond(id, Output::Ready);
                }
                Input::Plan(request) => {
                    let result = self.prepared.as_mut().expect("prepare precedes planning").run(request);
                    scope.respond(id, Output::Plan(result));
                }
            }
        }
    }

    pub(crate) struct PlannerEngine {
        kind: PlannerKind,
        generation: u64,
        worker: WorkerBridge<PlannerWorker>,
        results: Rc<RefCell<VecDeque<Output>>>,
        pending_since: Option<Instant>,
        prepared: bool,
    }

    impl PlannerEngine {
        pub(crate) fn new(kind: PlannerKind, track_index: usize, road: Road, initial_speed: f64) -> Self {
            let results = Rc::new(RefCell::new(VecDeque::new()));
            let callback_results = results.clone();
            let mut spawner = PlannerWorker::spawner();
            spawner
                .callback(move |result| callback_results.borrow_mut().push_back(result))
                .with_loader(true);
            let worker = spawner.spawn("planner-worker_loader.js");
            worker.send(Input::Prepare {
                track: track_index,
                dt: road.dt,
                initial_speed,
            });
            Self {
                kind,
                generation: 0,
                worker,
                results,
                pending_since: None,
                prepared: false,
            }
        }

        pub(crate) fn is_prepared(&mut self) -> bool {
            if matches!(self.results.borrow().front(), Some(Output::Ready)) {
                self.results.borrow_mut().pop_front();
                self.prepared = true;
            }
            self.prepared
        }

        pub(crate) fn reset(&mut self, kind: PlannerKind) {
            self.kind = kind;
            self.generation += 1;
            self.pending_since = None;
        }

        pub(crate) fn submit(&mut self, plan: PlanRequest) -> bool {
            if self.pending_since.is_some() {
                return false;
            }
            self.worker.send(Input::Plan(WorkerRequest {
                generation: self.generation,
                kind: self.kind,
                plan,
            }));
            self.pending_since = Some(Instant::now());
            true
        }

        pub(crate) fn poll(&mut self) -> Option<PlanResult> {
            while let Some(result) = self.results.borrow_mut().pop_front() {
                match result {
                    Output::Ready => self.prepared = true,
                    Output::Plan(result) if result.generation == self.generation => {
                        self.pending_since = None;
                        return Some(result.plan);
                    }
                    _ => {}
                }
            }
            None
        }

        pub(crate) fn is_slow(&self, planning_period_s: f64) -> bool {
            self.pending_since
                .is_some_and(|start| start.elapsed().as_secs_f64() > planning_period_s)
        }
    }

    pub(crate) fn register() {
        PlannerWorker::registrar().register();
    }
}

pub(crate) use platform::PlannerEngine;
#[cfg(target_family = "wasm")]
pub(crate) use platform::register;

#[cfg(all(test, not(target_family = "wasm")))]
mod tests {
    use std::time::Duration;

    use super::*;
    use crate::planning::test_road;

    struct GeometryProbe;
    impl Planner for GeometryProbe {
        fn plan(&mut self, ego: State, ctx: &Context) -> Vec<Control> {
            let before = crate::planning::latency::geometry_build_clocks();
            ctx.path().project(ego.position());
            ctx.road.closest_centerline_segment(ego.position());
            assert_eq!(crate::planning::latency::geometry_build_clocks(), before);
            vec![Control::default()]
        }
    }

    #[test]
    fn worker_keeps_prepared_geometry_across_ticks_windows_and_resets() {
        let track = crate::track::Track::from_catalog(1);
        let roads = track.prepared();
        let mut worker = PreparedPlanner {
            road: roads.planning.clone(),
            kind: PlannerKind::Straight,
            generation: 0,
            planner: Box::new(GeometryProbe),
        };
        let builds = crate::planning::latency::geometry_build_clocks();
        for tick in 0..20 {
            let road = roads.window(tick as f64 * 20.0, 40.0, 0.1);
            let result = worker.run(WorkerRequest {
                generation: u64::from(tick >= 10),
                kind: PlannerKind::Straight,
                plan: PlanRequest {
                    tick,
                    ego: State::default(),
                    road: road.view(),
                    actors: vec![],
                    horizon: 100,
                    compute_budget: ComputeBudget::NOMINAL,
                    diagnostics_enabled: false,
                },
            });
            assert_eq!(result.plan.tick, tick);
            assert_eq!(result.generation, u64::from(tick >= 10));
        }
        assert_eq!(crate::planning::latency::geometry_build_clocks(), builds);
        // Tick messages contain only a range and projection hint, never a Road.
        assert!(std::mem::size_of::<RoadView>() <= 48);
    }

    struct SlowPlanner;

    impl Planner for SlowPlanner {
        fn plan(&mut self, _ego: State, _ctx: &Context) -> Vec<Control> {
            std::thread::sleep(Duration::from_millis(50));
            vec![Control {
                acceleration: 1.0,
                curvature: 0.0,
            }]
        }
    }

    #[test]
    fn slow_planning_does_not_block_submission_and_reports_overrun() {
        let mut engine = PlannerEngine::with_planner(Box::new(SlowPlanner), test_road(&[[0.0, 0.0], [10.0, 0.0]]));
        let start = Instant::now();
        assert!(engine.submit(PlanRequest {
            tick: 7,
            ego: State::default(),
            road: test_road(&[[0.0, 0.0], [10.0, 0.0]]).view(),
            actors: vec![],
            horizon: 1,
            compute_budget: ComputeBudget::NOMINAL,
            diagnostics_enabled: false,
        }));
        assert!(start.elapsed() < Duration::from_millis(25));
        assert!(!engine.submit(PlanRequest {
            tick: 8,
            ego: State::default(),
            road: test_road(&[[0.0, 0.0], [10.0, 0.0]]).view(),
            actors: vec![],
            horizon: 1,
            compute_budget: ComputeBudget::NOMINAL,
            diagnostics_enabled: false,
        }));
        std::thread::sleep(Duration::from_millis(10));
        assert!(engine.is_slow(0.005));

        let result = engine.wait();
        assert_eq!(result.tick, 7);
        assert_eq!(result.controls[0].acceleration, 1.0);
    }
}
