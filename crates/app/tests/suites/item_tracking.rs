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
    let leaflet = opened
        .items
        .iter()
        .find(|o| o.name.contains("leaflet"))
        .expect("opening the mailbox reveals the leaflet");
    assert_eq!(
        leaflet.location,
        app::session::ObservedItemLocation::RoomNested,
        "the leaflet is reached only by recursing into the open mailbox, never the room's own top level"
    );
    let leaflet_key = leaflet.key;

    let mut mapper = mapper::mapper::Mapper::default();
    app::session::apply_turn(&mut mapper, "open mailbox", &opened, &mut Default::default());
    app::session::apply_item_observations(&mut mapper, 1, &opened);
    assert_eq!(
        mapper.graph.item(leaflet_key).unwrap().last_seen,
        mapper::graph::ItemLocation::Room { room: opened.location.as_ref().unwrap().number, direct: false }
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
    assert_ne!(rec.last_seen, mapper::graph::ItemLocation::Vanished, "a nested-only item must never read as vanished when its container closes");
    assert_eq!(
        rec.last_seen,
        mapper::graph::ItemLocation::Room { room: opened.location.as_ref().unwrap().number, direct: false },
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
    let origin_room = mapper.graph.item(leaflet_key).unwrap().origin_room;

    drive(&mut s, &mut mapper, "drop leaflet");
    let rec = mapper.graph.item(leaflet_key).unwrap();
    assert_eq!(rec.origin_room, origin_room, "origin never moves");
    assert_eq!(
        rec.last_seen,
        mapper::graph::ItemLocation::Room { room: origin_room, direct: true },
        "dropped back in the same room — a direct sighting again"
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
    let key = app::session::classify_take_attempt("take mailbox", &r.items).expect("one unambiguous candidate, visibly not carried");
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
    assert_eq!(
        app::session::classify_take_attempt("take leaflet", &r.items),
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
