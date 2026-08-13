use crate::common::types::position::Position;
use crate::simulation::State;

const MAX_POSITION_ERROR_M: f64 = 1.0;

fn matches(expected_next: State, ego: State) -> bool {
    Position::from(expected_next).distance(ego.into()) < MAX_POSITION_ERROR_M
}

/// Take a warm start only if the ego ended up where the previous plan predicted.
pub(crate) fn take_warm<T>(prev: &mut Option<T>, expected_next: State, ego: State) -> Option<T> {
    prev.take().filter(|_| matches(expected_next, ego))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn requires_the_ego_to_match_the_prediction() {
        let expected = State::default();
        let mut matching = Some(1);
        let mut diverged = Some(2);

        assert_eq!(
            take_warm(&mut matching, expected, {
                let mut state = expected;
                state.pose.position.x = 0.99;
                state
            }),
            Some(1)
        );
        assert_eq!(
            take_warm(&mut diverged, expected, {
                let mut state = expected;
                state.pose.position.x = 1.0;
                state
            }),
            None
        );
        assert_eq!(diverged, None);
    }
}
