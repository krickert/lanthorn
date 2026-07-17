//! Headless tile-map dump: read a babelmap map-dump file (`/map-dump` format),
//! realize one layer through `mapper::tiles::realize_layer`, rasterize it with
//! the real tile renderer into an offscreen buffer, and print the frame. The
//! visual iteration tool for the tile-map spike.
//!
//! Usage: `cargo run -p app --example tiles_dump -- map.txt 0`
//!
//! Only `ROOM <id> "<name>" pos=<x>,<y> … [layer=N]` and
//! `EDGE <origin> <DIR> <dest> [distorted]` lines are read; everything else
//! (comments, align/dropped notes) is ignored.

use app::render::tilemap::render_tile_map;
use app::state::AppState;
use mapper::direction::{parse_direction, Direction};
use mapper::graph::{MapGraph, RoomId};
use mapper::layer::LayerId;
use mapper::tiles::realize_layer;
use ratatui::buffer::Buffer;
use ratatui::layout::Rect;

/// A parsed `ROOM` line: id, name, grid pos, optional layer tag.
type ParsedRoom = (RoomId, String, (i32, i32), Option<LayerId>);

/// `ROOM 5 "Up a Tree" pos=-2,0 align=none [layer=N]` → (id, name, pos, layer).
fn parse_room(line: &str) -> Option<ParsedRoom> {
    let rest = line.strip_prefix("ROOM ")?;
    let (id, rest) = rest.split_once(' ')?;
    let id: RoomId = id.parse().ok()?;
    let rest = rest.trim_start().strip_prefix('"')?;
    let (name, rest) = rest.split_once('"')?;
    let mut pos = None;
    let mut layer = None;
    for tok in rest.split_whitespace() {
        if let Some(p) = tok.strip_prefix("pos=") {
            let (x, y) = p.split_once(',')?;
            pos = Some((x.parse().ok()?, y.parse().ok()?));
        } else if let Some(l) = tok.strip_prefix("layer=") {
            layer = l.parse().ok();
        }
    }
    Some((id, name.to_string(), pos?, layer))
}

/// `EDGE 78 N 131 [distorted]` → (origin, dir, dest, distorted). `?` is Unknown.
fn parse_edge(line: &str) -> Option<(RoomId, Direction, RoomId, bool)> {
    let mut tok = line.split_whitespace();
    tok.next()?; // "EDGE"
    let origin = tok.next()?.parse().ok()?;
    let dir = tok.next()?;
    let dir =
        if dir == "?" { Direction::Unknown } else { parse_direction(&dir.to_lowercase())? };
    let dest = tok.next()?.parse().ok()?;
    let distorted = tok.next() == Some("distorted");
    Some((origin, dir, dest, distorted))
}

fn main() {
    let mut args = std::env::args().skip(1);
    let Some(path) = args.next() else {
        eprintln!("usage: tiles_dump <map-dump.txt> [layer]");
        std::process::exit(2);
    };
    let layer: LayerId = args.next().map_or(0, |s| s.parse().expect("layer must be a number"));
    let text = std::fs::read_to_string(&path).unwrap_or_else(|e| {
        eprintln!("tiles_dump: {path}: {e}");
        std::process::exit(2);
    });

    let mut graph = MapGraph::new();
    for line in text.lines() {
        if let Some((id, name, pos, room_layer)) = parse_room(line) {
            graph.upsert_room(id, name);
            graph.set_pos(id, pos);
            if let Some(l) = room_layer {
                graph.set_room_layer(id, l);
            }
        }
    }
    for line in text.lines().filter(|l| l.starts_with("EDGE ")) {
        if let Some((origin, dir, dest, distorted)) = parse_edge(line) {
            graph.add_edge(origin, dir, dest);
            if distorted {
                // The dump carries the layout engine's distortion verdicts; replay
                // them so realization sees the same edges the app would.
                let idx = graph
                    .connections()
                    .iter()
                    .position(|c| c.origin == origin && c.dir == dir)
                    .expect("edge just added");
                graph.set_conn_distorted(idx, true);
            }
        }
    }

    let plan = realize_layer(&graph, layer);
    if plan.w == 0 || plan.h == 0 {
        println!("(layer {layer}: empty plan)");
        return;
    }
    eprintln!(
        "layer {layer}: {} rooms, {} conns, {}x{} tiles",
        plan.rooms.len(),
        plan.conns.len(),
        plan.w,
        plan.h
    );

    let area = Rect::new(0, 0, plan.w as u16, plan.h as u16);
    let state = AppState::default();
    let mut buf = Buffer::empty(area);
    render_tile_map(&plan, &graph, &state, area, &mut buf);
    for y in 0..area.height {
        let line: String =
            (0..area.width).map(|x| buf.cell((x, y)).unwrap().symbol()).collect::<String>();
        println!("{}", line.trim_end());
    }
}
