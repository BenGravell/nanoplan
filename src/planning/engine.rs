use web_time::Instant;

use crate::planning::{ComputeBudget, Context, Diagnostics, DiagnosticsData, Latency, Planner, PlannerKind, Span};
use crate::simulation::{Control, State};
use crate::track::Road;

#[cfg_attr(target_family = "wasm", derive(serde::Deserialize, serde::Serialize))]
pub(crate) struct PlanRequest {
    pub(crate) tick: u64,
    pub(crate) ego: State,
    pub(crate) road: Road,
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

fn run(planner: &mut dyn Planner, request: PlanRequest) -> PlanResult {
    let latency = Latency::default();
    let diagnostics = Diagnostics::default();
    let ctx = Context::new(
        &request.road,
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

#[cfg(not(target_family = "wasm"))]
mod platform {
    use std::sync::mpsc::{Receiver, SyncSender, TryRecvError, sync_channel};

    use super::*;

    pub(crate) struct PlannerEngine {
        requests: SyncSender<PlanRequest>,
        results: Receiver<PlanResult>,
        pending_since: Option<Instant>,
    }

    impl PlannerEngine {
        pub(crate) fn new(kind: PlannerKind) -> Self {
            Self::with_planner(kind.build())
        }

        pub(crate) fn with_planner(mut planner: Box<dyn Planner>) -> Self {
            let (requests, request_rx) = sync_channel::<PlanRequest>(1);
            let (result_tx, results) = sync_channel(1);
            std::thread::Builder::new()
                .name("nanoplan-planner".into())
                .spawn(move || {
                    while let Ok(request) = request_rx.recv() {
                        if result_tx.send(run(&mut *planner, request)).is_err() {
                            break;
                        }
                    }
                })
                .expect("planner worker thread should start");
            Self {
                requests,
                results,
                pending_since: None,
            }
        }

        pub(crate) fn submit(&mut self, request: PlanRequest) -> bool {
            if self.pending_since.is_some() || self.requests.try_send(request).is_err() {
                return false;
            }
            self.pending_since = Some(Instant::now());
            true
        }

        pub(crate) fn poll(&mut self) -> Option<PlanResult> {
            match self.results.try_recv() {
                Ok(result) => {
                    self.pending_since = None;
                    Some(result)
                }
                Err(TryRecvError::Empty) => None,
                Err(TryRecvError::Disconnected) => {
                    self.pending_since = None;
                    None
                }
            }
        }

        pub(crate) fn is_slow(&self, planning_period_s: f64) -> bool {
            self.pending_since
                .is_some_and(|start| start.elapsed().as_secs_f64() > planning_period_s)
        }

        pub(crate) fn wait(&mut self) -> PlanResult {
            let result = self.results.recv().expect("planner worker should return a result");
            self.pending_since = None;
            result
        }
    }
}

#[cfg(target_family = "wasm")]
mod platform {
    use std::cell::RefCell;
    use std::collections::VecDeque;
    use std::rc::Rc;

    use gloo_worker::{HandlerId, Registrable, Spawnable, Worker, WorkerBridge, WorkerScope};

    use super::*;

    #[derive(serde::Deserialize, serde::Serialize)]
    struct WorkerRequest {
        kind: PlannerKind,
        plan: PlanRequest,
    }

    struct PlannerWorker {
        kind: Option<PlannerKind>,
        planner: Option<Box<dyn Planner>>,
    }

    impl Worker for PlannerWorker {
        type Message = ();
        type Input = WorkerRequest;
        type Output = PlanResult;

        fn create(_scope: &WorkerScope<Self>) -> Self {
            Self {
                kind: None,
                planner: None,
            }
        }

        fn update(&mut self, _scope: &WorkerScope<Self>, _message: Self::Message) {}

        fn received(&mut self, scope: &WorkerScope<Self>, request: Self::Input, id: HandlerId) {
            if self.kind != Some(request.kind) {
                self.kind = Some(request.kind);
                self.planner = Some(request.kind.build());
            }
            scope.respond(
                id,
                run(
                    &mut **self.planner.as_mut().expect("worker planner should be initialized"),
                    request.plan,
                ),
            );
        }
    }

    pub(crate) struct PlannerEngine {
        kind: PlannerKind,
        worker: WorkerBridge<PlannerWorker>,
        results: Rc<RefCell<VecDeque<PlanResult>>>,
        pending_since: Option<Instant>,
    }

    impl PlannerEngine {
        pub(crate) fn new(kind: PlannerKind) -> Self {
            let results = Rc::new(RefCell::new(VecDeque::new()));
            let callback_results = results.clone();
            let mut spawner = PlannerWorker::spawner();
            spawner
                .callback(move |result| callback_results.borrow_mut().push_back(result))
                .with_loader(true);
            Self {
                kind,
                worker: spawner.spawn("planner-worker_loader.js"),
                results,
                pending_since: None,
            }
        }

        pub(crate) fn submit(&mut self, request: PlanRequest) -> bool {
            if self.pending_since.is_some() {
                return false;
            }
            self.worker.send(WorkerRequest {
                kind: self.kind,
                plan: request,
            });
            self.pending_since = Some(Instant::now());
            true
        }

        pub(crate) fn poll(&mut self) -> Option<PlanResult> {
            let result = self.results.borrow_mut().pop_front();
            if result.is_some() {
                self.pending_since = None;
            }
            result
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
        let mut engine = PlannerEngine::with_planner(Box::new(SlowPlanner));
        let start = Instant::now();
        assert!(engine.submit(PlanRequest {
            tick: 7,
            ego: State::default(),
            road: test_road(&[[0.0, 0.0], [10.0, 0.0]]),
            actors: vec![],
            horizon: 1,
            compute_budget: ComputeBudget::NOMINAL,
            diagnostics_enabled: false,
        }));
        assert!(start.elapsed() < Duration::from_millis(25));
        assert!(!engine.submit(PlanRequest {
            tick: 8,
            ego: State::default(),
            road: test_road(&[[0.0, 0.0], [10.0, 0.0]]),
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
