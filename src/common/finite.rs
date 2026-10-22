//! Finiteness checks for values and groups of values.

use crate::common::types::{Acceleration, Control, Pose, Position, Speed, State, Vec2d};

pub(crate) trait IsFinite {
    fn is_finite(&self) -> bool;
}

/// Return the input unchanged when every component is finite.
/// Accepts individual values, arrays, and tuples of elements; tuples may be nested.
pub(crate) fn check_finite<T: IsFinite>(args: T) -> Option<T> {
    args.is_finite().then_some(args)
}

macro_rules! impl_is_finite {
    ($($ty:ty),+ $(,)?) => {
        $(impl IsFinite for $ty {
            fn is_finite(&self) -> bool {
                <$ty>::is_finite(*self)
            }
        })+
    };
}

impl_is_finite!(f32, f64, Vec2d, Position, Pose, State, Control);

impl IsFinite for Speed {
    fn is_finite(&self) -> bool {
        Vec2d::is_finite(**self)
    }
}

impl IsFinite for Acceleration {
    fn is_finite(&self) -> bool {
        Vec2d::is_finite(**self)
    }
}

impl<T: IsFinite, const N: usize> IsFinite for [T; N] {
    fn is_finite(&self) -> bool {
        self.iter().all(IsFinite::is_finite)
    }
}

macro_rules! impl_tuple_is_finite {
    ($($ty:ident:$field:tt),+) => {
        impl<$($ty: IsFinite),+> IsFinite for ($($ty,)+) {
            fn is_finite(&self) -> bool {
                true $(&& self.$field.is_finite())+
            }
        }
    };
}

impl_tuple_is_finite!(A:0);
impl_tuple_is_finite!(A:0, B:1);
impl_tuple_is_finite!(A:0, B:1, C:2);
impl_tuple_is_finite!(A:0, B:1, C:2, D:3);
impl_tuple_is_finite!(A:0, B:1, C:2, D:3, E:4);
impl_tuple_is_finite!(A:0, B:1, C:2, D:3, E:4, F:5);
impl_tuple_is_finite!(A:0, B:1, C:2, D:3, E:4, F:5, G:6);
impl_tuple_is_finite!(A:0, B:1, C:2, D:3, E:4, F:5, G:6, H:7);
impl_tuple_is_finite!(A:0, B:1, C:2, D:3, E:4, F:5, G:6, H:7, I:8);
impl_tuple_is_finite!(A:0, B:1, C:2, D:3, E:4, F:5, G:6, H:7, I:8, J:9);
impl_tuple_is_finite!(A:0, B:1, C:2, D:3, E:4, F:5, G:6, H:7, I:8, J:9, K:10);
impl_tuple_is_finite!(A:0, B:1, C:2, D:3, E:4, F:5, G:6, H:7, I:8, J:9, K:10, L:11);

#[cfg(test)]
mod tests {
    use super::*;
    use crate::common::types::Position;

    #[test]
    fn accepts_mixed_tuples_and_checks_nested_arrays() {
        let args = (1.0_f32, State::default(), [2.0_f64, 3.0]);
        assert_eq!(check_finite(args), Some(args));
        assert_eq!(check_finite(4.0_f64), Some(4.0));
        assert!(check_finite((Speed::new(1.0, 2.0), Acceleration::new(3.0, 4.0))).is_some());
        for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(check_finite((1.0_f32, State::default(), [2.0, value])).is_none());
            assert!(check_finite((Speed::new(value, 2.0),)).is_none());
            assert!(check_finite((Acceleration::new(3.0, value),)).is_none());
        }
    }

    #[test]
    fn check_finite_rejects_each_nonfinite_component() {
        let sample = |v: [f64; 6]| {
            check_finite((
                State::from((Position::new(v[0], v[1]), v[2], v[3])),
                Control {
                    acceleration: v[4],
                    curvature: v[5],
                },
            ))
        };
        assert!(sample([1.0; 6]).is_some());
        for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            for component in 0..6 {
                let mut values = [1.0; 6];
                values[component] = value;
                assert!(sample(values).is_none(), "component {component}: {value}");
            }
        }
    }
}
