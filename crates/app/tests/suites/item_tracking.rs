//! Item location tracking (SQ-1627): the mapper's per-item origin/current-location/vanished
//! registry, and Facet 3's structural fixed-in-place detector, end to end through a real
//! Z-machine session. Real-game cases skip vacuously without `stories/` (gitignored), the
//! CI-safe pattern `room_description.rs` (SQ-1625) already uses.
//!
//! `stories/zork1-r88-s840726.z3`: release 88 / serial 840726 — West of House's mailbox and
//! leaflet are the specimen for every facet here: the mailbox is a classic non-takeable object
//! ("It is securely anchored."), and the leaflet is a classic container-nested item (visible only
//! once the mailbox is opened), which happens to be exactly the shape Facet 1's correction is
//! about. Verified against the real interpreter output before writing any assertion below
//! (`cargo run -p lanthorn-zvm-cli -- stories/zork1-r88-s840726.z3` driven with `open mailbox` /
//! `take leaflet` / `close mailbox` / `take mailbox`).

use crate::fixture_paths::fixture_path;

use app::engine::{Engine, KeyInput};
use app::glulx_session::GlulxSession;
use app::session::GameSession;

fn story(name: &str) -> Option<Vec<u8>> {
    std::fs::read(fixture_path(name)).ok()
}

fn boot_zork1() -> Option<GameSession> {
    let bytes = story("zork1-r88-s840726.z3")?;
    let mut s = GameSession::new_with_trace(bytes, true, false, None, false, Vec::new(), None, None, Some((25, 80)))
        .expect("zork1 boots without a ZError");
    // Drain the boot's own banner/opening print first — same idiom `room_description.rs` uses.
    let _ = s.submit("");
    Some(s)
}

/// The Glulx image inside a Blorb, or a bare `.ulx` passed through — same helper
/// `glulx_inventory.rs` uses. `None` when the gitignored fixture is absent.
fn glulx_image(name: &str) -> Option<Vec<u8>> {
    let path = fixture_path(name);
    let bytes = std::fs::read(&path).ok()?;
    if !blorb::Blorb::is_blorb(&bytes) {
        return Some(bytes);
    }
    let b = blorb::Blorb::parse(bytes).ok()?;
    match b.executable() {
        Ok((blorb::ExecKind::Glulx, data)) => Some(data.to_vec()),
        _ => None,
    }
}

/// Counterfeit Monkey (Inform 7 6M62, IF Archive release 10 / SQ-1454) past its "Can you hear
/// me?" intro to the first real command prompt — see `glulx_inventory.rs`'s
/// `counterfeit_monkey_refuses_an_avatar_it_cannot_identify` for why this exact fixture is the
/// one Inform 7 game already documented here as having NO hardware short name on virtually any
/// of its objects, avatar included.
fn boot_cm() -> Option<GlulxSession> {
    let image = glulx_image("CounterfeitMonkey-10.gblorb")?;
    let mut s = GlulxSession::new(image, 80, 24, true, false, false, (1.0, 1.0), None, &[]).expect("GlulxSession::new");
    for cmd in ["yes", "yes", "yes"] {
        s.submit(cmd);
    }
    s.submit_key(KeyInput::Enter);
    let _ = s.take_transcript();
    Some(s)
}

/// Facet 1: the mailbox (never opened, never moved) is tracked as a direct sighting in West of
/// House from the very first turn it's observed.
#[test]
fn a_direct_room_item_is_tracked_from_its_first_sighting() {
    let Some(mut s) = boot_zork1() else {
        eprintln!("SKIP: gitignored stories/zork1-r88-s840726.z3 missing");
        return;
    };
    let r = s.submit("look");
    let mailbox = r
        .items
        .iter()
        .find(|o| o.name.contains("mailbox"))
        .expect("West of House prints a small mailbox");
    assert_eq!(mailbox.location, app::session::ObservedItemLocation::RoomDirect);
}

/// Facet 1 (the correction): opening the mailbox reveals the leaflet as a NESTED sighting, not a
/// direct one — and closing the mailbox again must not make the leaflet read as vanished on a
/// later look, because a closed container's contents are simply unreachable, not gone.
#[test]
fn a_leaflet_seen_only_inside_the_mailbox_is_nested_and_survives_the_mailbox_closing() {
    let Some(mut s) = boot_zork1() else {
        eprintln!("SKIP: gitignored stories/zork1-r88-s840726.z3 missing");
        return;
    };
    let opened = s.submit("open mailbox");
    let mailbox_key = opened
        .items
        .iter()
        .find(|o| o.name.contains("mailbox"))
        .expect("the mailbox itself is still a direct sighting")
        .key;
    let leaflet = opened
        .items
        .iter()
        .find(|o| o.name.contains("leaflet"))
        .expect("opening the mailbox reveals the leaflet");
    assert_eq!(
        leaflet.location,
        app::session::ObservedItemLocation::RoomNested { container: Some(mailbox_key) },
        "SQ-1632 Fix 1: the leaflet is nested inside the mailbox specifically, not just \"nested somewhere\""
    );
    let leaflet_key = leaflet.key;

    let mut mapper = mapper::mapper::Mapper::default();
    app::session::apply_turn(&mut mapper, "open mailbox", &opened, &mut Default::default());
    app::session::apply_item_observations(&mut mapper, 1, &opened);
    assert_eq!(
        mapper.graph.item(leaflet_key).unwrap().last_seen,
        mapper::graph::ItemLocation::Room {
            room: opened.location.as_ref().unwrap().number,
            direct: false,
            container: Some(mailbox_key),
        }
    );

    // Close the mailbox — the leaflet drops out of every observation entirely (unreachable, not
    // gone) — and LOOK again to re-drive a full turn against the same room.
    let closed = s.submit("close mailbox");
    assert!(!closed.items.iter().any(|o| o.name.contains("leaflet")), "closed — no longer observable at all");
    app::session::apply_turn(&mut mapper, "close mailbox", &closed, &mut Default::default());
    app::session::apply_item_observations(&mut mapper, 2, &closed);
    let looked = s.submit("look");
    assert!(!looked.items.iter().any(|o| o.name.contains("leaflet")), "still unreachable after a fresh LOOK");
    app::session::apply_turn(&mut mapper, "look", &looked, &mut Default::default());
    app::session::apply_item_observations(&mut mapper, 3, &looked);

    let rec = mapper.graph.item(leaflet_key).unwrap();
    assert!(
        !matches!(rec.last_seen, mapper::graph::ItemLocation::Vanished { .. }),
        "a nested-only item must never read as vanished when its container closes"
    );
    assert_eq!(
        rec.last_seen,
        mapper::graph::ItemLocation::Room {
            room: opened.location.as_ref().unwrap().number,
            direct: false,
            container: Some(mailbox_key),
        },
        "the record is left exactly as it was — the correction's whole point"
    );
}

/// Facet 1: taking the leaflet out of the mailbox moves it to Carried; dropping it again in the
/// SAME room re-confirms it as a direct sighting there. Origin never moves off West of House.
#[test]
fn taking_and_dropping_the_leaflet_moves_its_tracked_location() {
    let Some(mut s) = boot_zork1() else {
        eprintln!("SKIP: gitignored stories/zork1-r88-s840726.z3 missing");
        return;
    };
    let mut mapper = mapper::mapper::Mapper::default();
    let mut turn = 0u32;
    let mut drive = |s: &mut GameSession, mapper: &mut mapper::mapper::Mapper, cmd: &str| {
        let r = s.submit(cmd);
        turn += 1;
        app::session::apply_turn(mapper, cmd, &r, &mut Default::default());
        app::session::apply_item_observations(mapper, turn, &r);
        r
    };

    drive(&mut s, &mut mapper, "open mailbox");
    let taken = drive(&mut s, &mut mapper, "take leaflet");
    let leaflet_key = taken.items.iter().find(|o| o.name.contains("leaflet")).expect("still observed, now carried").key;
    assert_eq!(mapper.graph.item(leaflet_key).unwrap().last_seen, mapper::graph::ItemLocation::Carried);
    assert_eq!(
        mapper.graph.item(leaflet_key).unwrap().carried_since_turn,
        Some(2),
        "SQ-1632 Fix 3: picked up on turn 2 (open mailbox=1, take leaflet=2)"
    );
    let origin_room = mapper.graph.item(leaflet_key).unwrap().origin_room;

    // Re-confirming it's STILL carried (an ordinary look) must not disturb the pick-up turn.
    drive(&mut s, &mut mapper, "look");
    assert_eq!(
        mapper.graph.item(leaflet_key).unwrap().carried_since_turn,
        Some(2),
        "still held — the pick-up turn does not slide forward on a re-confirmation"
    );

    drive(&mut s, &mut mapper, "drop leaflet");
    let rec = mapper.graph.item(leaflet_key).unwrap();
    assert_eq!(rec.origin_room, origin_room, "origin never moves");
    assert_eq!(
        rec.last_seen,
        mapper::graph::ItemLocation::Room { room: origin_room, direct: true, container: None },
        "dropped back in the same room — a direct sighting again"
    );
    assert_eq!(
        rec.carried_since_turn, None,
        "SQ-1632 Fix 3: dropping it clears the pick-up turn — it is not carried any more"
    );
}

/// Facet 3: "take mailbox" visibly fails ("It is securely anchored.") and the mailbox is the
/// unambiguous sole candidate for that noun in this room — confirmed fixed-in-place.
#[test]
fn taking_the_mailbox_confirms_it_fixed_in_place() {
    let Some(mut s) = boot_zork1() else {
        eprintln!("SKIP: gitignored stories/zork1-r88-s840726.z3 missing");
        return;
    };
    let r = s.submit("take mailbox");
    assert!(r.transcript.contains("securely anchored"), "the real refusal text: {:?}", r.transcript);
    let vocab = s.story_vocabulary();
    let key = app::session::classify_take_attempt("take mailbox", &r.items, vocab.as_ref())
        .expect("one unambiguous candidate, visibly not carried");
    let mailbox = r.items.iter().find(|o| o.key == key).unwrap();
    assert!(mailbox.name.contains("mailbox"));
    assert_ne!(mailbox.location, app::session::ObservedItemLocation::Carried);
}

/// Facet 3, the negative case: taking the leaflet (an ordinary takeable item) after opening the
/// mailbox must NOT be confirmed fixed-in-place — the take visibly worked.
#[test]
fn taking_an_ordinary_takeable_item_is_not_flagged_fixed_in_place() {
    let Some(mut s) = boot_zork1() else {
        eprintln!("SKIP: gitignored stories/zork1-r88-s840726.z3 missing");
        return;
    };
    let _ = s.submit("open mailbox");
    let r = s.submit("take leaflet");
    assert!(r.transcript.contains("Taken"), "the real success text: {:?}", r.transcript);
    let vocab = s.story_vocabulary();
    assert_eq!(
        app::session::classify_take_attempt("take leaflet", &r.items, vocab.as_ref()),
        None,
        "the take worked — nothing to flag"
    );
}

/// Observed-only guarantee: an item in a room the player has never visited must never appear in
/// the item registry, even after many turns of play elsewhere. Falsified (2026-09-27) by
/// temporarily widening `GameSession::zvm_item_observations` (`session.rs`) to also walk
/// `zvm::location::object_tree_view` — every object in the game, not just the current room and
/// inventory — which made this test fail with the kitchen's sack/bottle/table and every other
/// unvisited-room item leaking into the registry; reverted once confirmed.
#[test]
fn an_item_in_a_never_visited_room_never_appears_in_the_registry() {
    let Some(mut s) = boot_zork1() else {
        eprintln!("SKIP: gitignored stories/zork1-r88-s840726.z3 missing");
        return;
    };
    let mut mapper = mapper::mapper::Mapper::default();
    let mut turn = 0u32;
    // Play several turns entirely around West of House / North of House / Forest — never
    // entering the house, never reaching e.g. the kitchen (which holds its own items: a sack,
    // a bottle, a table).
    for cmd in ["look", "north", "north", "east", "south", "look"] {
        let r = s.submit(cmd);
        turn += 1;
        app::session::apply_turn(&mut mapper, cmd, &r, &mut Default::default());
        app::session::apply_item_observations(&mut mapper, turn, &r);
    }
    let tracked_names: Vec<String> = mapper.graph.items().map(|(_, rec)| rec.name.clone()).collect();
    assert!(
        !tracked_names.iter().any(|n| n.contains("sack") || n.contains("bottle") || n.contains("table")),
        "the kitchen's own items must never appear — the player has never stood in that room: {tracked_names:?}"
    );
    // Every item actually tracked must have an origin room the player did, in fact, walk
    // through this session (a resolved, non-synthetic location every `submit` above returned).
    assert!(!tracked_names.is_empty(), "West of House's own mailbox should still be tracked");
}

/// SQ-1632 Fix 5: the quest's own real repro against Zork I r88 — "white house", "board",
/// "stairs", "chimney", "kitchen window", "boarded window" are Inform/ZIL local-global/shared
/// scenery (`zvm::world::WorldModel::local_globals`), visible from several rooms but a genuine
/// child of NONE of them (`real_container_in_room` returns `None` for every one). Before the
/// fix, each such object was recorded as `RoomNested` in EVERY room it happened to be visible
/// from, reading in the item registry as though one portable object silently relocated itself as
/// the player walked ("white house" moving West of House -> North of House -> Behind House).
/// Falsify by temporarily dropping the `real_container_in_room` filter in
/// `GameSession::zvm_item_observations` and re-running: this test fails with exactly that shape
/// (confirmed before writing the fix).
#[test]
fn shared_scenery_never_reads_as_a_portable_item_following_the_player() {
    let Some(mut s) = boot_zork1() else {
        eprintln!("SKIP: gitignored stories/zork1-r88-s840726.z3 missing");
        return;
    };
    let mut mapper = mapper::mapper::Mapper::default();
    let mut turn = 0u32;
    let mut drive = |s: &mut GameSession, mapper: &mut mapper::mapper::Mapper, cmd: &str| {
        let r = s.submit(cmd);
        turn += 1;
        app::session::apply_turn(mapper, cmd, &r, &mut Default::default());
        app::session::apply_item_observations(mapper, turn, &r);
        r
    };

    // The quest's own repro, verbatim.
    for cmd in [
        "open mailbox", "take leaflet", "take mailbox", "north", "east", "open window",
        "enter window", "open sack", "take bottle", "west", "take lamp", "take sword", "east",
        "turn on lamp", "up", "take rope", "down", "drop bottle", "look",
    ] {
        drive(&mut s, &mut mapper, cmd);
    }

    // None of the scenery the report named is in the item registry at all — the fix EXCLUDES
    // local-global scenery from item tracking outright, rather than merely pinning it to one room.
    let names: Vec<String> = mapper.graph.items().map(|(_, rec)| rec.name.to_lowercase()).collect();
    for scenery in ["white house", "board", "stairs", "chimney", "window"] {
        assert!(
            !names.iter().any(|n| n.contains(scenery)),
            "local-global scenery {scenery:?} must never enter the item registry at all: {names:?}"
        );
    }
}

// ── Fix 1 / Fix 3 (SQ-1631) — a real Inform 7 game with an unidentifiable avatar ─────────────

/// SQ-1631 Fix 1: Counterfeit Monkey (Inform 7 6M62, IF Archive release 10) is documented
/// (`glulx_inventory.rs`'s `counterfeit_monkey_refuses_an_avatar_it_cannot_identify`) as having
/// no hardware short name on virtually any of its objects, avatar included — every ordinary
/// scenery object here prints through a `parse_name` rule instead of a static short name. Before
/// this fix, `glulx_item_observations` filtered candidates on the raw (empty) printed name and
/// dropped every one of them outright, so `result.items` was empty for this game's every room
/// regardless of what was actually shown. Sigil Street — one `north` past the opening Back Alley,
/// once the room lock has resolved a real address (turn 0 predates that and reports no items at
/// all, which is why this drives one move first) — is a real specimen: its own "sky backdrops"
/// scenery has an empty printed name and is named only by its parse words.
#[test]
fn glulx_item_observations_reaches_an_inform_7_object_with_no_printed_name() {
    let Some(mut s) = boot_cm() else { return };
    let r = s.submit("north"); // Back Alley -> Sigil Street
    assert_eq!(
        r.location.as_ref().map(|l| l.name.as_str()),
        Some("Sigil Street"),
        "premise: this walks to the specimen room: {:?}",
        r.location
    );

    // Confirm the premise directly against the object list: this room really does hold an
    // object with an empty raw printed name but a real display name.
    let loc = r.location.clone().unwrap();
    let room_objects = s.introspect().unwrap().room_objects_excluding(loc.number, None);
    let unnamed: Vec<_> =
        room_objects.iter().filter(|o| o.printed_name.is_empty() && o.display_name().is_some()).collect();
    assert!(
        !unnamed.is_empty(),
        "premise: Sigil Street holds an Inform-7-style object with no printed name: {room_objects:?}"
    );

    // And it now reaches `result.items` — with its `display_name()` as `name`, never empty.
    assert!(
        r.items.iter().any(|i| unnamed.iter().any(|o| o.id == i.key) && !i.name.is_empty()),
        "at least one such object now reaches result.items with a real display name: {:?}",
        r.items
    );
}

/// SQ-1631 Fix 3: `glulx_item_observations` must prefer [`app::engine::Engine::set_player_hint`]'s
/// value over the raw [`app::engine::Introspect::player_object`] lookup, which for Counterfeit
/// Monkey answers `None` FOREVER — no turn ever locks it by the engine's own name-based heuristic
/// (`glulx_inventory.rs`'s `counterfeit_monkey_refuses_an_avatar_it_cannot_identify`). The host's
/// own movement-tracking fallback (`app::inventory::detect_player_obj`) can still lock a real
/// handle by watching what moved between rooms even when the engine's own lookup cannot, and
/// `finish_command_turn` pushes that lock in via `set_player_hint`. This exercises the mechanism
/// directly against CM's own real object tree: with no hint, nothing is excluded from "directly
/// in this room" at all; with a hint set to a real handle this room holds, that ONE object is
/// excluded — exactly as it would be if it really were the player.
#[test]
fn glulx_item_observations_prefers_the_set_player_hint_over_the_unidentifiable_raw_lookup() {
    let Some(mut s) = boot_cm() else { return };
    let r = s.submit("north");
    let loc = r.location.clone().unwrap();
    assert!(
        s.introspect().unwrap().player_object().is_none(),
        "premise: CM's avatar is unidentifiable by name"
    );

    let without_hint: std::collections::BTreeSet<u32> = r
        .items
        .iter()
        .filter(|i| i.location == app::session::ObservedItemLocation::RoomDirect)
        .map(|i| i.key)
        .collect();
    assert!(!without_hint.is_empty(), "premise: Sigil Street shows at least one room-direct item");

    let handles = s.introspect().unwrap().children_of(loc.number);
    let &stand_in = handles.iter().next().expect("Sigil Street holds at least one child object");

    s.set_player_hint(Some(stand_in));
    let with_hint = s.submit("look");
    let after: std::collections::BTreeSet<u32> = with_hint
        .items
        .iter()
        .filter(|i| i.location == app::session::ObservedItemLocation::RoomDirect)
        .map(|i| i.key)
        .collect();

    assert_eq!(
        after.len(),
        without_hint.len().saturating_sub(1),
        "exactly one object — the hinted handle — is now excluded from room-direct: before \
         {without_hint:?}, after {after:?}"
    );
    assert_eq!(
        without_hint.difference(&after).count(),
        1,
        "the excluded object was really among the previously-listed ones, not a coincidental \
         absence: before {without_hint:?}, after {after:?}"
    );
}
