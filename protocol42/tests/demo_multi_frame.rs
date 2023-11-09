//! Parse multiple telemetry frames generated from 42's Demo project

use protocol42::*;

const TELEM_BUFFER: &str = include_str!("../test_fixtures/demo_multi_frame.txt");

#[test]
fn demo_multi_frame() {
    let mut msgs = Vec::new();
    let mut buf = String::new();

    for l in TELEM_BUFFER.split_inclusive('\n') {
        buf.push_str(l);
        match parse_telemetry(&buf) {
            Ok((rest, telem)) => {
                assert_eq!(rest, "");
                msgs.push(telem);
                buf.clear();
            }
            Err(e) => {
                if !e.is_failure() {
                    continue;
                } else {
                    panic!("Parse error. {e}");
                }
            }
        }
    }

    assert_eq!(msgs.len(), 2);
    let m0 = &msgs[0];
    let m1 = &msgs[1];

    let dt = m1.timestamp - m0.timestamp;
    assert_eq!(dt.num_milliseconds(), 100);

    assert_eq!(m0.spacecrafts.len(), m1.spacecrafts.len());
    assert_eq!(m0.orbits.len(), m1.orbits.len());
    assert_eq!(m0.worlds.len(), m1.worlds.len());
}
