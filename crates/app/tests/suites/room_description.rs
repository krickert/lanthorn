//! Room description capture (SQ-1625): the mapper's most-recent-description-per-room feature,
//! end to end through a real Z-machine session. Real-game cases skip vacuously without
//! `stories/` (gitignored), the CI-safe pattern documented in `fixture_paths.rs`.
//!
//! Only the Z-machine (Tier 3, bounded scope) is exercised here with a real story — Scott's
//! `room_description_text` and Glk's `StoryScan` extension are unit-tested directly in their own
//! crates (`crates/scott/src/vm.rs`, `crates/app/src/glk_backend.rs`), where a synthetic fixture
//! is exact and immediate rather than dependent on a commercial story's own prose.

use crate::fixture_paths::fixture_path;

use app::session::GameSession;

fn story(name: &str) -> Option<Vec<u8>> {
    std::fs::read(fixture_path(name)).ok()
}

/// `stories/zork1-r88-s840726.z3`: release 88 / serial 840726, a Version 3 story with the
/// ordinary status-line-plus-one-scrolling-window convention (`docs/internals/interpreter.md`).
///
/// The BOOT's own opening-room print never reaches `description` at all — `Engine::seed_turn`
/// deliberately drains only `location`/`quit`/`erase_lower` (see its own doc: "the three OTHER
/// per-turn facts... were looked for and are not there"), never the transcript `drain_turn`
/// builds `description` from. So this drives the FIRST real turn (`submit`, exactly the path
/// `finish_command_turn` uses) rather than the boot frame — an explicit LOOK, which is one of
/// this feature's two named triggers in its own right, not a stand-in for the other.
#[test]
fn single_window_zmachine_arrival_and_look_capture_a_description() {
    let Some(bytes) = story("zork1-r88-s840726.z3") else {
        eprintln!("SKIP: gitignored stories/zork1-r88-s840726.z3 missing");
        return;
    };
    let mut s = GameSession::new_with_trace(bytes, true, false, None, false, Vec::new(), None, None, Some((25, 80)))
        .expect("zork1 boots without a ZError");
    assert!(s.machine.screen.v6.is_none(), "zork1 is not a v6 story");
    // Drain and discard the boot's own banner/opening print first (the same idiom
    // `declared_exit.rs`'s `Play::for_story` uses) — `Engine::seed_turn` never touches
    // `description` at all (see this test's own doc), so the boot's undrained buffer would
    // otherwise land INSIDE the first real `submit`'s transcript and double-print the room.
    let _ = s.submit("");

    // Explicit LOOK in the boot's starting room.
    let r1 = s.submit("look");
    let loc1 = r1.location.clone().expect("zork1 names a room");
    assert_eq!(loc1.name, "West of House");
    let desc1 = r1.description.clone().expect("an explicit LOOK must capture a description");
    assert!(!desc1.is_empty());
    assert!(!desc1.contains("West of House"), "the heading itself must not reappear in the body: {desc1:?}");
    assert!(!desc1.to_lowercase().contains("obvious exits"), "no exits-section leakage: {desc1:?}");

    // A second LOOK re-captures the SAME room's text, replacing rather than accumulating.
    let r2 = s.submit("look");
    let desc2 = r2.description.expect("a second LOOK re-captures the description");
    assert_eq!(desc2, desc1, "West of House's own text is unchanged, and not doubled, by a second look");

    // A genuine ARRIVAL — walking north into a different, differently-described room —
    // captures ITS OWN text, distinct from West of House's.
    let r3 = s.submit("north");
    let loc3 = r3.location.expect("the walk north names a room");
    assert_ne!(loc3.name, "West of House", "the walk actually left the starting room");
    let desc3 = r3.description.expect("a genuine arrival must capture a description");
    assert_ne!(desc3, desc1, "a different room's description must not be the old room's leftover text");
}

/// `stories/zork0-r393-s890714.z6`: a Version 6 story — Zork Zero, one of the four the quest's
/// own bound names explicitly. Whatever this turn's transcript looks like, the v6 gate must
/// leave `description` `None` outright: no extraction attempt, no garbage, no panic.
///
/// Zork Zero's own opening is a scripted banquet-hall cutscene that always ends in a death
/// (`****  You have died  ****`) a few turns in, so this drives exactly that far and no
/// further — real gameplay proper starts only after answering the RESTART/RESTORE/UNDO/QUIT
/// prompt, which is outside what this bound needs to demonstrate. The decisive falsifying case —
/// a v6-shaped transcript that WOULD produce a wrong description if the gate were removed — is
/// `session::tests::zvm_room_description_is_gated_off_for_a_v6_story`, a synthetic unit test:
/// this game's own opening never happens to hand `detect_location_with` a resolved room at all
/// (`location` stays `None` throughout the cutscene), which cannot exercise the gate on its own.
#[test]
fn v6_zmachine_story_never_captures_a_description() {
    let Some(bytes) = story("zork0-r393-s890714.z6") else {
        eprintln!("SKIP: gitignored stories/zork0-r393-s890714.z6 missing");
        return;
    };
    let mut s = GameSession::new_with_trace(bytes, true, false, None, false, Vec::new(), None, None, None)
        .expect("zork0 boots without a ZError");
    assert!(s.machine.screen.v6.is_some(), "zork0 is the v6 fixture this bound is about");

    // Advance a few real turns (a keypress past the intro, then a couple of lines through the
    // banquet-hall cutscene) — the gate must hold on every DRAINED turn, not merely on an
    // untested boot frame, and must never panic however the cutscene's own windows are laid out.
    let _ = s.submit_char(b' ');
    for _ in 0..5 {
        let r = s.submit("look");
        assert_eq!(r.description, None, "still gated: {:?}", r.transcript);
    }
}
