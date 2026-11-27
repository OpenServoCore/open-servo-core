//! The board facts the servo publishes in CALIB (protocol sec 5.5), read
//! off the client's simulated servo, which the board file's host mirror
//! seeds: the words step 0 keeps, and the block a capture's meta carries.

mod common;

use futures_lite::future::block_on;
use osc_measure::board::{Board, Words};
use osc_measure::capture::{meta, tel};
use osc_measure::state;

#[test]
fn the_board_read_off_the_fake_servo_is_the_dev_board() {
    let d = common::descriptor();
    let want = common::board_record().registers;
    for (what, (mut c, _)) in [
        ("off the line", common::factory()),
        ("calibrated", common::seeded()),
    ] {
        let s = block_on(state::read(&mut c, common::ID, &d)).expect("state");
        assert_eq!(Words::of(&s, &d).expect("words"), want, "{what}");
        assert_eq!(Board::of(&s, &d).expect("board"), common::board(), "{what}");
        let line = block_on(tel::line(&mut c, common::ID)).expect("line");
        let m = meta::servo(&s, &d, &line);
        let words = serde_json::to_value(&want).expect("json");
        for name in meta::BOARD {
            assert_eq!(m["board"][name], words[name], "{what}: {name}");
        }
    }
}
