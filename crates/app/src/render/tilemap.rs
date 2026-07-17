//! Tile-map rasterizer (tile map spike): draws a `mapper::tiles::TilePlan` into
//! the map pane, one tile per terminal cell. Every glyph comes from
//! `SymbolSet::tiles` and every style from a `map.tile.*` ColorScheme field —
//! nothing here is hard-coded, matching the classic renderer's theming rules.

use mapper::graph::{MapGraph, RoomId};
use mapper::tiles::{DoorKind, FeatureKind, Tile, TilePlan};
use ratatui::buffer::Buffer;
use ratatui::layout::Rect;

use crate::render::{put_char, put_str};
use crate::state::AppState;
use crate::symbols::TileGlyphs;

/// The tile shown at the pane's top-left corner. Mirrors `render_map`'s pan
/// state (`state.scroll` cells + `state.char_pan` chars; at this zoom both are
/// tile units, and positive `char_pan` shifts content right like the classic
/// renderer's `+ char_pan` offset), clamped so the plan can always be panned
/// fully into view and never entirely off-screen.
fn tile_origin(plan: &TilePlan, state: &AppState, area: Rect) -> (i32, i32) {
    let max_x = (plan.w as i32 - i32::from(area.width)).max(0);
    let max_y = (plan.h as i32 - i32::from(area.height)).max(0);
    (
        (state.scroll.0 - state.char_pan.0).clamp(0, max_x),
        (state.scroll.1 - state.char_pan.1).clamp(0, max_y),
    )
}

/// The glyph for a door tile: two-way `∩`, one-way a direction triangle (a
/// diagonal renders as its vertical cardinal), stub `?`.
fn door_glyph(kind: DoorKind, g: &TileGlyphs) -> char {
    use mapper::direction::Direction as D;
    match kind {
        DoorKind::TwoWay => g.door,
        DoorKind::Stub(_) => g.door_stub,
        DoorKind::OneWay(dir) => match dir {
            D::N | D::NE | D::NW => g.door_n,
            D::S | D::SE | D::SW => g.door_s,
            D::E => g.door_e,
            D::W => g.door_w,
            _ => g.door,
        },
    }
}

fn feature_glyph(kind: FeatureKind, g: &TileGlyphs) -> char {
    match kind {
        FeatureKind::StairsUp => g.stairs_up,
        FeatureKind::StairsDown => g.stairs_down,
        FeatureKind::PortalIn => g.portal_in,
        FeatureKind::PortalOut => g.portal_out,
    }
}

/// Draw `plan` into `buf` for `area` (1 tile = 1 cell, clipped and panned via
/// [`tile_origin`]). Void tiles leave the buffer untouched so the background
/// shows through. The current room (from `graph.current()`) gets the player
/// marker at its floor centre; `state.show_room_numbers` adds a `#id` label
/// centred on each floor (shifted one row up on the current room).
pub fn render_tile_map(
    plan: &TilePlan,
    graph: &MapGraph,
    state: &AppState,
    area: Rect,
    buf: &mut Buffer,
) {
    let (ox, oy) = tile_origin(plan, state, area);
    let g = &state.symbols.tiles;
    let cs = &state.colors;

    let x1 = plan.w.min(ox as usize + area.width as usize);
    let y1 = plan.h.min(oy as usize + area.height as usize);
    for ty in oy as usize..y1 {
        for tx in ox as usize..x1 {
            let (ch, style) = match plan.get(tx, ty) {
                Tile::Void => continue,
                Tile::Wall => (g.wall, cs.tile_wall),
                Tile::Floor { .. } => (g.floor, cs.tile_floor),
                Tile::Corridor { .. } => (g.corridor, cs.tile_corridor),
                Tile::Door { kind, .. } => (door_glyph(kind, g), cs.tile_door),
                Tile::Bridge { .. } => (g.bridge, cs.tile_bridge),
                Tile::Feature { kind, .. } => (feature_glyph(kind, g), cs.tile_stairs),
            };
            let sx = i32::from(area.x) + tx as i32 - ox;
            let sy = i32::from(area.y) + ty as i32 - oy;
            put_char(buf, sx, sy, ch, style, area);
        }
    }

    // Overlays: room-number labels and the player marker, on top of the floor.
    let current = graph.current();
    for room in &plan.rooms {
        let f = room.floor;
        let (cx, cy) = ((f.x + f.w / 2) as i32, (f.y + f.h / 2) as i32);
        let is_current = current == Some(room.id);
        if state.show_room_numbers {
            let label = format!("#{}", room.id);
            let len = label.chars().count();
            // The current room's centre belongs to '@': the label moves one row up.
            let ly = if is_current { cy - 1 } else { cy };
            if len <= f.w && ly >= f.y as i32 {
                let lx = f.x as i32 + (f.w as i32 - len as i32) / 2;
                let (sx, sy) = (i32::from(area.x) + lx - ox, i32::from(area.y) + ly - oy);
                put_str(buf, sx, sy, &label, cs.tile_room_number, area);
            }
        }
        if is_current {
            let (sx, sy) = (i32::from(area.x) + cx - ox, i32::from(area.y) + cy - oy);
            put_char(buf, sx, sy, g.player, cs.tile_player, area);
        }
    }
}

/// Screen-space floor rect per room, clipped to `area`, for mouse hit-testing.
/// Uses the same [`tile_origin`] as [`render_tile_map`] so clicks land exactly
/// on what was drawn. Fully off-screen rooms are omitted.
pub fn tile_room_screen_rects(
    plan: &TilePlan,
    state: &AppState,
    area: Rect,
) -> Vec<(RoomId, Rect)> {
    let (ox, oy) = tile_origin(plan, state, area);
    let mut rects = Vec::with_capacity(plan.rooms.len());
    for room in &plan.rooms {
        let f = room.floor;
        let sx = i32::from(area.x) + f.x as i32 - ox;
        let sy = i32::from(area.y) + f.y as i32 - oy;
        let x0 = sx.max(i32::from(area.x));
        let y0 = sy.max(i32::from(area.y));
        let x1 = (sx + f.w as i32).min(i32::from(area.right()));
        let y1 = (sy + f.h as i32).min(i32::from(area.bottom()));
        if x1 <= x0 || y1 <= y0 {
            continue;
        }
        rects.push((room.id, Rect::new(x0 as u16, y0 as u16, (x1 - x0) as u16, (y1 - y0) as u16)));
    }
    rects
}

#[cfg(test)]
mod tests {
    use super::*;
    use mapper::direction::Direction;
    use mapper::layer::MAIN_LAYER;
    use mapper::tiles::realize_layer;

    /// Rooms 1 ⇄ 2 adjacent (shared wall + door), room 3 two columns east of 2
    /// (walled corridor). Player in room 1.
    fn three_room_graph() -> MapGraph {
        let mut g = MapGraph::new();
        for (id, pos) in [(1u16, (0, 0)), (2, (1, 0)), (3, (3, 0))] {
            g.upsert_room(id, format!("r{id}"));
            g.set_pos(id, pos);
        }
        g.add_edge(1, Direction::E, 2);
        g.add_edge(2, Direction::W, 1);
        g.add_edge(2, Direction::E, 3);
        g.add_edge(3, Direction::W, 2);
        g.set_current(1);
        g
    }

    fn frame(buf: &Buffer, area: Rect) -> String {
        let mut out = String::new();
        for y in area.y..area.bottom() {
            for x in area.x..area.right() {
                out.push_str(buf.cell((x, y)).unwrap().symbol());
            }
            out.push('\n');
        }
        out
    }

    fn room(plan: &TilePlan, id: RoomId) -> &mapper::tiles::TileRoom {
        plan.rooms.iter().find(|r| r.id == id).unwrap()
    }

    #[test]
    fn snapshot_three_rooms_with_shared_door_and_corridor() {
        let g = three_room_graph();
        let plan = realize_layer(&g, MAIN_LAYER);
        let area = Rect::new(0, 0, 40, 13);
        assert!(plan.w <= area.width as usize && plan.h <= area.height as usize,
            "fixture plan {}x{} must fit the {}x{} test area", plan.w, plan.h, area.width, area.height);
        let state = AppState::default(); // scroll (0,0), numbers off
        let mut buf = Buffer::empty(area);
        render_tile_map(&plan, &g, &state, area, &mut buf);
        let got = frame(&buf, area);

        // Verified by eye: rooms 1|2 share a wall column with one ∩ door; room 3
        // hangs off a walled corridor with ∩ doors at both ends; '@' sits on room
        // 1's floor centre; Void stays blank (rows are padded to the 40-cell area).
        let expected = concat!(
            "                                        \n",
            "                                        \n",
            "        █████████   ███████             \n",
            "  ███████·······█   █·····█             \n",
            "  █·····█·······█████·····█             \n",
            "  █··@··∩·······∩···∩·····█             \n",
            "  █·····█·······█████·····█             \n",
            "  ███████·······█   █·····█             \n",
            "        █████████   ███████             \n",
            "                                        \n",
            "                                        \n",
            "                                        \n",
            "                                        \n",
        );
        assert_eq!(got, expected, "frame changed:\n{got}");

        // Structural properties, independent of the exact frame:
        let (r1, r2) = (room(&plan, 1), room(&plan, 2));
        assert_eq!(r1.bounds.right(), r2.bounds.x, "rooms 1|2 share exactly one wall column");
        let shared_x = r1.bounds.right() as u16;
        let doors_in_shared: Vec<u16> = (0..area.height)
            .filter(|&y| buf.cell((shared_x, y)).unwrap().symbol() == "∩")
            .collect();
        assert_eq!(doors_in_shared.len(), 1, "one ∩ door in the shared wall");
        // '@' on room 1's floor centre.
        let f = r1.floor;
        let (cx, cy) = ((f.x + f.w / 2) as u16, (f.y + f.h / 2) as u16);
        assert_eq!(buf.cell((cx, cy)).unwrap().symbol(), "@");
        // The 2→3 corridor: a '·' corridor tile walled with '█' above and below.
        let (tx, ty) = (0..plan.h)
            .flat_map(|y| (0..plan.w).map(move |x| (x, y)))
            .find(|&(x, y)| matches!(plan.get(x, y), Tile::Corridor { .. }))
            .expect("corridor tiles exist between rooms 2 and 3");
        assert_eq!(buf.cell((tx as u16, ty as u16)).unwrap().symbol(), "·");
        assert_eq!(buf.cell((tx as u16, ty as u16 - 1)).unwrap().symbol(), "█");
        assert_eq!(buf.cell((tx as u16, ty as u16 + 1)).unwrap().symbol(), "█");
    }

    #[test]
    fn room_numbers_show_and_shift_up_in_the_current_room() {
        let g = three_room_graph();
        let plan = realize_layer(&g, MAIN_LAYER);
        let area = Rect::new(0, 0, 40, 13);
        let mut state = AppState::default();
        state.show_room_numbers = true;
        let mut buf = Buffer::empty(area);
        render_tile_map(&plan, &g, &state, area, &mut buf);

        let at = |x: u16, y: u16| buf.cell((x, y)).unwrap().symbol().to_string();
        // Room 2 (not current): "#2" centred on the floor's centre row.
        let f2 = room(&plan, 2).floor;
        let cy2 = (f2.y + f2.h / 2) as u16;
        let lx2 = f2.x as u16 + (f2.w as u16 - 2) / 2;
        assert_eq!(at(lx2, cy2), "#");
        assert_eq!(at(lx2 + 1, cy2), "2");
        // Room 1 (current): '@' keeps the centre, "#1" one row up.
        let f1 = room(&plan, 1).floor;
        let (cx1, cy1) = ((f1.x + f1.w / 2) as u16, (f1.y + f1.h / 2) as u16);
        assert_eq!(at(cx1, cy1), "@");
        let lx1 = f1.x as u16 + (f1.w as u16 - 2) / 2;
        assert_eq!(at(lx1, cy1 - 1), "#");
        assert_eq!(at(lx1 + 1, cy1 - 1), "1");
    }

    #[test]
    fn screen_rects_track_the_render_origin() {
        let g = three_room_graph();
        let plan = realize_layer(&g, MAIN_LAYER);

        // Unscrolled, plan fully visible: each rect IS the room's floor rect.
        let area = Rect::new(0, 0, 40, 13);
        let state = AppState::default();
        let rects = tile_room_screen_rects(&plan, &state, area);
        assert_eq!(rects.len(), plan.rooms.len());
        for (id, r) in &rects {
            let f = room(&plan, *id).floor;
            assert_eq!(
                (r.x, r.y, r.width, r.height),
                (f.x as u16, f.y as u16, f.w as u16, f.h as u16),
                "room {id} rect matches its floor at origin (0,0)"
            );
        }

        // Scrolled by (1,1) with the pane smaller than the plan (no clamping):
        // every rect shifts by exactly (-1,-1).
        let small = Rect::new(0, 0, (plan.w - 2) as u16, (plan.h - 2) as u16);
        let mut scrolled = AppState::default();
        scrolled.scroll = (1, 1);
        let base = tile_room_screen_rects(&plan, &AppState::default(), small);
        let moved = tile_room_screen_rects(&plan, &scrolled, small);
        for (id, r) in &moved {
            let (_, b) = base.iter().find(|(bid, _)| bid == id).expect("room visible in both");
            assert_eq!((r.x, r.y), (b.x - 1, b.y - 1), "room {id} shifted with the scroll");
        }
    }
}
