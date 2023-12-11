use fsw_lib::scenario::config::Config;
use std::{collections::HashSet, fs, path::Path};

const CONFIG_FILES: &[&str] = &[
    "all_ground_stations.toml",
    "example.toml",
    "point_failures.toml",
];

#[test]
fn example_scenario_config_file_list_matches_expected() {
    let cfg_files: HashSet<String> = fs::read_dir("../scenarios")
        .unwrap()
        .map(|d| d.unwrap().file_name().into_string().unwrap())
        .collect();
    let expected: HashSet<String> = CONFIG_FILES.iter().map(|f| f.to_string()).collect();
    assert_eq!(cfg_files, expected, "Example scenarios directory is missing an expected config file or contains a new config file that should be tested");
}

#[test]
fn example_scenario_config_files_parse() {
    let dir = Path::new("../scenarios");
    for cfg_file in CONFIG_FILES {
        let p = dir.join(cfg_file);
        let _cfg = Config::load(p);
    }
}
