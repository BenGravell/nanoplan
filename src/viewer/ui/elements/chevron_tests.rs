use super::chevron_animation;

#[test]
fn landing_chevron_pulses_and_bounces_horizontally() {
    let (center, normal) = chevron_animation(0.0);
    let (shoulder, _) = chevron_animation(1.0 / 6.0);
    let (right, large) = chevron_animation(1.0 / 3.0);
    let (left, small) = chevron_animation(1.0);

    assert!(large > normal && small < normal);
    assert!(right > center && left < center);
    assert!(shoulder > right * 0.9);
}
