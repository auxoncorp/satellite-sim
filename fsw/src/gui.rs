use kiss3d::{
    camera::{ArcBall, Camera},
    context::Texture,
    event::{Action, Key, WindowEvent},
    light::Light,
    resource::{MaterialManager, Mesh, MeshManager, TextureManager},
    scene::{Object, SceneNode},
    text::Font,
    window::Window,
};
use na::{Isometry3, Point2, Point3, Quaternion, UnitQuaternion, Vector2, Vector3, Vector4};
use std::{
    cell::RefCell,
    collections::HashMap,
    fmt::Write,
    path::{Path, PathBuf},
    rc::Rc,
};
use types42::prelude::{World, WorldKind};

use crate::{
    ground_station::GroundStationId,
    interruptor::Interruptor,
    satellite::{SatCatId, SATELLITE_IDS},
    sim_info::SimulationInfo,
};

pub const SYSTEM_SCALE: f64 = 1000000.0;
pub const SIM_INFO_TEXT_SCALE: f32 = 50.0;
pub const SAT_INFO_TEXT_SCALE: f32 = 20.0;
pub const IR_EVENT_INFO_TEXT_SCALE: f32 = 16.0;

pub type SharedGuiState = Rc<RefCell<GuiState>>;
pub type GroundTruthId = i64;

const EARTH_TEXTURE_JPG: &[u8] = include_bytes!("../textures/2k_earth_daymap.jpg");
const MOON_TEXTURE_JPG: &[u8] = include_bytes!("../textures/2k_moon.jpg");

const SAT_NOMINAL_RGB: [f32; 3] = [1.0, 1.0, 0.0];
const SAT_ERROR_RGB: [f32; 3] = [1.0, 0.0, 0.0];

// NOTE: not to scale
const SAT_OBJ_SCALE: Vector3<f32> = Vector3::new(0.4, 0.4, 0.4);

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum RenderContext {
    /// Simulation
    Sim,
    /// FSM
    Fsm,
}

pub struct GuiState {
    window: Window,
    cam: ArcBall,
    font: Rc<Font>,
    text_origin: Point2<f32>,
    text_color: Point3<f32>,
    text_buf: String,

    sat_model_resources: Option<ModelResources>,

    interruptor: Interruptor,
    paused: bool,
    step: bool,
    sim_info_visibility: bool,
    xyz_axis_visibility: bool,
    sun_pointing_vector_visibility: bool,
    sat_info_visibility: bool,
    ir_event_info_visibility: bool,
    mode: RenderContext,
    ratelimiter_active: bool,

    celestial_body_nodes: HashMap<WorldKind, SceneNode>,
    unit_sun_vector: Vector3<f32>,

    sat_nodes: HashMap<SatCatId, SceneNode>,
    sat_highlight_nodes: HashMap<SatCatId, SceneNode>,
    scanner_cam_nodes: HashMap<SatCatId, SceneNode>,
    focus_cam_nodes: HashMap<SatCatId, SceneNode>,
    ir_event_ndoes: HashMap<GroundTruthId, SceneNode>,
    relay_ground_station_nodes: HashMap<GroundStationId, SceneNode>,
}

impl GuiState {
    pub fn new_shared(
        win_title: &str,
        start_paused: bool,
        interruptor: Interruptor,
        sat_model_path: Option<PathBuf>,
    ) -> SharedGuiState {
        Rc::new(RefCell::new(Self::new(
            win_title,
            start_paused,
            interruptor,
            sat_model_path,
        )))
    }

    pub fn new(
        win_title: &str,
        start_paused: bool,
        interruptor: Interruptor,
        sat_model_path: Option<PathBuf>,
    ) -> Self {
        let mut window = Window::new(win_title);

        let sat_model_resources = if let Some(sat_model_path) = &sat_model_path {
            let mut resources = Vec::new();

            assert!(
                sat_model_path.exists(),
                "satellite model path doesn't exist"
            );
            let mtl_dir_path = sat_model_path.parent().expect("Material parent directory");

            let ops = tobj::GPU_LOAD_OPTIONS;
            let (models, materials) =
                tobj::load_obj(sat_model_path, &ops).expect("Failed to load satellite obj");
            let materials = materials.expect("Failed to load satellite obj materials");

            assert!(models.len() > 1);
            for (idx, model) in models.iter().enumerate() {
                assert!(model.mesh.positions.len() % 3 == 0);
                assert_eq!(model.mesh.positions.len(), model.mesh.normals.len());
                assert_eq!(model.mesh.positions.len() / 3, model.mesh.indices.len());

                let mesh_name = format!("sat_model_{idx}");
                let coords = model
                    .mesh
                    .positions
                    .chunks_exact(3)
                    .map(|p| Point3::new(p[0], p[1], p[2]))
                    .collect();
                let faces = model
                    .mesh
                    .indices
                    .chunks_exact(3)
                    .map(|p| Point3::new(p[0] as u16, p[1] as u16, p[2] as u16))
                    .collect();
                let normals = model
                    .mesh
                    .normals
                    .chunks_exact(3)
                    .map(|p| Vector3::new(p[0], p[1], p[2]))
                    .collect();
                let uvs = if model.mesh.texcoords.is_empty() {
                    None
                } else {
                    Some(
                        model
                            .mesh
                            .texcoords
                            .chunks_exact(2)
                            .map(|p| Point2::new(p[0], p[1]))
                            .collect(),
                    )
                };
                let mesh = Mesh::new(
                    coords,
                    faces,
                    Some(normals),
                    uvs,
                    false, /*dynamic_draw*/
                );
                let mesh = Rc::new(RefCell::new(mesh));

                MeshManager::get_global_manager(|mm| {
                    mm.add(mesh.clone(), &mesh_name);
                });

                let (color, texture) = if let Some(mtl_idx) = model.mesh.material_id {
                    let mtl = materials.get(mtl_idx).unwrap();
                    let color = mtl.diffuse.unwrap();
                    if let Some(texture_file) = mtl.diffuse_texture.as_ref() {
                        let p = mtl_dir_path.join(texture_file);
                        let tex = TextureManager::get_global_manager(|tm| tm.add(&p, texture_file));
                        (color, tex.into())
                    } else if let Some(texture_file) = mtl.ambient_texture.as_ref() {
                        let p = mtl_dir_path.join(texture_file);
                        let tex = TextureManager::get_global_manager(|tm| tm.add(&p, texture_file));
                        (color, tex.into())
                    } else {
                        (color, None)
                    }
                } else {
                    ([1.0, 1.0, 1.0], None)
                };

                resources.push((mesh, color, texture));
            }

            Some(ModelResources(resources))
        } else {
            None
        };

        let eye = Vector3::new(1.0, 2.0, 1.0).scale(30.0);
        let at = Point3::origin();
        let mut cam = ArcBall::new(eye.into(), at);
        cam.set_up_axis(Vector3::z());

        window.set_light(Light::StickToCamera);
        window.set_framerate_limit(None);

        let mut earth = window.add_sphere((WorldKind::EARTH_RADIUS / SYSTEM_SCALE) as _);
        earth.set_surface_rendering_activation(true);
        earth.set_texture_from_memory(EARTH_TEXTURE_JPG, "earth");

        // Align the texture with the ECEF world frame
        let rx =
            UnitQuaternion::from_scaled_axis(Vector3::new(-std::f32::consts::FRAC_PI_2, 0.0, 0.0));
        earth.append_rotation(&rx);
        let rz = UnitQuaternion::from_scaled_axis(Vector3::new(0.0, 0.0, -std::f32::consts::PI));
        earth.append_rotation(&rz);

        let mut celestial_body_nodes = HashMap::new();
        celestial_body_nodes.insert(WorldKind::Earth, earth);

        println!("----------------------------------------------");
        println!("|                 GUI key map                |");
        println!("----------------------------------------------");
        println!("'p'      : snapshot : pauses and writes snapshot.png to the CWD");
        println!("SPACE    : pause");
        println!("'n'      : step");
        println!("'m'      : mode : render every sim step (default) or FSM step");
        println!("'i'      : toggle sim info visibility");
        println!("'f'      : toggle focus camera viewing volume visibility");
        println!("'s'      : toggle scanner camera viewing volume visibility");
        println!("'e'      : toggle IR event visibility");
        println!("'j'      : toggle sun pointing vector visibility");
        println!("'k'      : toggle XYZ axis visibility");
        println!("'l'      : toggle satellite info visibility");
        println!("'o'      : toggle IR event label visibility");
        println!("'g'      : toggle relay ground station visibility");
        println!("'r'      : toggle earth visibility");
        println!("'RSHIFT' : toggle time-per-step rate limiter");
        println!("ESC      : exit");
        println!("----------------------------------------------");

        Self {
            window,
            cam,
            font: Font::default(),
            text_origin: Point2::origin(),
            text_color: Point3::new(1.0, 1.0, 1.0),
            text_buf: String::with_capacity(1024),
            interruptor,
            paused: start_paused,
            step: false,
            sim_info_visibility: true,
            xyz_axis_visibility: true,
            sun_pointing_vector_visibility: true,
            sat_info_visibility: true,
            ir_event_info_visibility: true,
            mode: RenderContext::Sim,
            ratelimiter_active: true,
            sat_model_resources,
            celestial_body_nodes,
            unit_sun_vector: Vector3::zeros(),
            sat_nodes: Default::default(),
            sat_highlight_nodes: Default::default(),
            scanner_cam_nodes: Default::default(),
            focus_cam_nodes: Default::default(),
            ir_event_ndoes: Default::default(),
            relay_ground_station_nodes: Default::default(),
        }
    }

    pub fn ratelimiter_active(&self) -> bool {
        self.ratelimiter_active
    }

    /// Draw ECEF xyz axis lines
    fn draw_xyz_lines(&mut self) {
        self.window.draw_line(
            &Point3::origin(),
            &Point3::new(100.0, 0.0, 0.0),
            &Point3::new(1.0, 0.0, 0.0),
        );
        self.window.draw_line(
            &Point3::origin(),
            &Point3::new(0.0, 100.0, 0.0),
            &Point3::new(0.0, 1.0, 0.0),
        );
        self.window.draw_line(
            &Point3::origin(),
            &Point3::new(0.0, 0.0, 100.0),
            &Point3::new(0.0, 0.0, 1.0),
        );
    }

    /// Draw the sun-pointing vector
    fn draw_sun_pointing_vec(&mut self) {
        let p = self.unit_sun_vector.into();
        self.window
            .draw_line(&Point3::origin(), &p, &Point3::new(1.0, 1.0, 0.0));
    }

    /// Draw sim info text
    fn draw_sim_info(&mut self, sim_info: &SimulationInfo) {
        self.text_buf.clear();
        write!(&mut self.text_buf, "{}", sim_info.timestamp).unwrap();
        self.text_origin.y = 0.0;
        self.window.draw_text(
            &self.text_buf,
            &self.text_origin,
            SIM_INFO_TEXT_SCALE,
            &self.font,
            &self.text_color,
        );

        self.text_buf.clear();
        write!(
            &mut self.text_buf,
            "Sim iteration: {}",
            sim_info.sim_iteration
        )
        .unwrap();
        self.text_origin.y += SIM_INFO_TEXT_SCALE;
        self.window.draw_text(
            &self.text_buf,
            &self.text_origin,
            SIM_INFO_TEXT_SCALE,
            &self.font,
            &self.text_color,
        );

        self.text_buf.clear();
        write!(
            &mut self.text_buf,
            "FSW: {} s",
            sim_info.relative_time.as_secs()
        )
        .unwrap();
        self.text_origin.y += SIM_INFO_TEXT_SCALE;
        self.window.draw_text(
            &self.text_buf,
            &self.text_origin,
            SIM_INFO_TEXT_SCALE,
            &self.font,
            &self.text_color,
        );

        self.text_buf.clear();
        write!(
            &mut self.text_buf,
            "FSM iteration: {}",
            sim_info.fsw_iteration
        )
        .unwrap();
        self.text_origin.y += SIM_INFO_TEXT_SCALE;
        self.window.draw_text(
            &self.text_buf,
            &self.text_origin,
            SIM_INFO_TEXT_SCALE,
            &self.font,
            &self.text_color,
        );

        self.text_buf.clear();
        write!(&mut self.text_buf, "Mode: {}", self.mode.as_str()).unwrap();
        self.text_origin.y += SIM_INFO_TEXT_SCALE;
        self.window.draw_text(
            &self.text_buf,
            &self.text_origin,
            SIM_INFO_TEXT_SCALE,
            &self.font,
            &self.text_color,
        );
    }

    fn draw_sat_labels(&mut self) {
        for (satcat_id, n) in self.sat_nodes.iter() {
            let label = SATELLITE_IDS
                .iter()
                .find_map(|id| {
                    if id.satcat_id == *satcat_id {
                        Some(id.name)
                    } else {
                        None
                    }
                })
                .unwrap();
            let pos_w = n.data().local_translation().vector.into();
            let win_size = Vector2::new(self.window.size()[0] as f32, self.window.size()[1] as f32);
            let mut pos = self.cam.project(&pos_w, &win_size);
            pos.y = win_size.y - pos.y;

            self.text_buf.clear();
            write!(&mut self.text_buf, "{}", label).unwrap();
            self.window.draw_text(
                &self.text_buf,
                &pos.into(),
                SAT_INFO_TEXT_SCALE,
                &self.font,
                &self.text_color,
            );
        }
    }

    fn draw_ir_event_labels(&mut self) {
        for (gt_id, n) in self.ir_event_ndoes.iter() {
            let pos_w = n.data().local_translation().vector.into();
            let win_size = Vector2::new(self.window.size()[0] as f32, self.window.size()[1] as f32);
            let mut pos = self.cam.project(&pos_w, &win_size);
            pos.y = win_size.y - pos.y;

            self.text_buf.clear();
            write!(&mut self.text_buf, "{}", gt_id).unwrap();
            self.window.draw_text(
                &self.text_buf,
                &pos.into(),
                IR_EVENT_INFO_TEXT_SCALE,
                &self.font,
                &self.text_color,
            );
        }
    }

    /// Returns false if the window should be closed
    pub fn render<F>(
        &mut self,
        ctx: RenderContext,
        sim_info: &SimulationInfo,
        force: bool,
        paused_cb: F,
    ) -> bool
    where
        F: Fn(),
    {
        if !force && (ctx != self.mode) {
            return true;
        }

        loop {
            if self.xyz_axis_visibility {
                self.draw_xyz_lines();
            }

            if self.sun_pointing_vector_visibility {
                self.draw_sun_pointing_vec();
            }

            if self.sim_info_visibility {
                self.draw_sim_info(sim_info);
            }

            if self.sat_info_visibility {
                self.draw_sat_labels();
            }

            if self.ir_event_info_visibility {
                self.draw_ir_event_labels();
            }

            let kr = self.window.render_with_camera(&mut self.cam);

            for event in self.window.events().iter() {
                if let WindowEvent::Key(key, Action::Press, _) = event.value {
                    match key {
                        Key::I => {
                            self.sim_info_visibility = !self.sim_info_visibility;
                        }
                        Key::R => {
                            if let Some(n) = self.celestial_body_nodes.get_mut(&WorldKind::Earth) {
                                n.set_visible(!n.is_visible());
                            }
                        }
                        Key::F => {
                            self.focus_cam_nodes
                                .values_mut()
                                .for_each(|n| n.set_visible(!n.is_visible()));
                        }
                        Key::S => {
                            self.scanner_cam_nodes
                                .values_mut()
                                .for_each(|n| n.set_visible(!n.is_visible()));
                        }
                        Key::E => {
                            self.ir_event_ndoes
                                .values_mut()
                                .for_each(|n| n.set_visible(!n.is_visible()));
                        }
                        Key::G => {
                            self.relay_ground_station_nodes
                                .values_mut()
                                .for_each(|n| n.set_visible(!n.is_visible()));
                        }
                        Key::J => {
                            self.sun_pointing_vector_visibility =
                                !self.sun_pointing_vector_visibility;
                        }
                        Key::K => {
                            self.xyz_axis_visibility = !self.xyz_axis_visibility;
                        }
                        Key::L => {
                            self.sat_info_visibility = !self.sat_info_visibility;
                        }
                        Key::O => {
                            self.ir_event_info_visibility = !self.ir_event_info_visibility;
                        }
                        Key::M => {
                            self.mode = self.mode.swap();
                        }
                        Key::N => {
                            self.step = true;
                        }
                        Key::P => {
                            self.paused = true;
                            self.step = false;

                            let img = self.window.snap_image();
                            let img_path = Path::new("snapshot.png");
                            println!("Writing snapshot to '{}'", img_path.display());
                            img.save(img_path).unwrap();
                        }
                        Key::Space => {
                            self.paused = !self.paused;
                            self.step = false;

                            if self.paused {
                                println!("Paused at t = {}", sim_info.timestamp);
                            }
                        }
                        Key::RShift => {
                            self.ratelimiter_active = !self.ratelimiter_active;
                        }
                        _ => (),
                    }
                }
            }

            if !self.paused || !kr || self.step || self.interruptor.is_set() {
                self.step = false;
                break kr && !self.interruptor.is_set();
            } else {
                // Paused
                paused_cb();
            }
        }
    }

    pub fn update_celestial_bodies(&mut self, worlds: &HashMap<WorldKind, World>) {
        let earth = worlds.get(&WorldKind::Earth).expect("Missing sun world");
        let moon = worlds.get(&WorldKind::Luna).expect("Missing sun world");

        let sun_vector = earth.pos_h * -1.0;
        let unit_sun_vector = sun_vector.normalize().scale(50.0);
        self.unit_sun_vector.x = unit_sun_vector.x as _;
        self.unit_sun_vector.y = unit_sun_vector.y as _;
        self.unit_sun_vector.z = unit_sun_vector.z as _;

        let moon_pos_w = moon.pos_h - earth.pos_h;

        self.celestial_body_nodes
            .entry(WorldKind::Luna)
            .and_modify(|n| {
                n.set_local_translation(scale_v3(&moon_pos_w).into());
            })
            .or_insert_with(|| {
                let mut n = self
                    .window
                    .add_sphere((WorldKind::LUNA_RADIUS / SYSTEM_SCALE) as _);
                n.set_surface_rendering_activation(true);
                n.set_texture_from_memory(MOON_TEXTURE_JPG, "sun");
                n.set_local_translation(scale_v3(&moon_pos_w).into());
                n
            });
    }

    pub fn update_satellite(
        &mut self,
        satcat_id: SatCatId,
        has_errors: bool,
        pos_w: &Vector3<f64>,
        _vel_w: &Vector3<f64>,
    ) {
        let pos = scale_v3(pos_w);

        let n = self
            .sat_nodes
            .entry(satcat_id)
            .and_modify(|n| {
                n.set_local_translation(scale_v3(pos_w).into());
            })
            .or_insert_with(|| {
                let mut n = create_sat_model_node(&mut self.window, &self.sat_model_resources);
                n.set_local_translation(pos.into());
                n
            });

        if self.sat_model_resources.is_some() {
            let alignment_rot_x =
                UnitQuaternion::from_scaled_axis(Vector3::new(std::f32::consts::PI, 0.0, 0.0));
            let alignment_rot_z = UnitQuaternion::from_scaled_axis(Vector3::new(
                0.0,
                0.0,
                -std::f32::consts::FRAC_PI_2,
            ));
            let rot = UnitQuaternion::look_at_rh(&pos, &Vector3::z());
            n.set_local_rotation(rot.inverse() * alignment_rot_x * alignment_rot_z);
        }

        // Draw a red wire box around the satellite if it has any error bits set
        if has_errors {
            self.sat_highlight_nodes
                .entry(satcat_id)
                .and_modify(|n| {
                    n.set_local_translation(scale_v3(pos_w).into());
                })
                .or_insert_with(|| {
                    let mut n = self.window.add_cube(1.5, 1.5, 1.5);
                    n.set_color(SAT_ERROR_RGB[0], SAT_ERROR_RGB[1], SAT_ERROR_RGB[2]);
                    n.set_lines_width(0.45);
                    n.set_surface_rendering_activation(false);
                    n.set_local_translation(scale_v3(pos_w).into());
                    n
                });
        } else if let Some(mut n) = self.sat_highlight_nodes.remove(&satcat_id) {
            self.window.remove_node(&mut n);
        }
    }

    pub fn update_satellite_scanner_camera(
        &mut self,
        satcat_id: SatCatId,
        viewing_volume_radius: f64,
        viewing_volume_height: f64,
        iso: &Isometry3<f64>,
    ) {
        self.scanner_cam_nodes
            .entry(satcat_id)
            .and_modify(|n| {
                n.set_local_scale(
                    scale(viewing_volume_radius * 2.0),
                    scale(viewing_volume_height),
                    scale(viewing_volume_radius * 2.0),
                );
                let iso = scale_iso(iso);
                n.set_local_transformation(iso);
            })
            .or_insert_with(|| {
                let mut n = self
                    .window
                    .add_cone(scale(viewing_volume_radius), scale(viewing_volume_height));
                n.set_color(0.8, 0.8, 0.0);
                n.set_points_size(1.0);
                n.set_lines_width(1.0);
                n.set_surface_rendering_activation(false);
                let iso = scale_iso(iso);
                n.set_local_transformation(iso);
                n
            });
    }

    pub fn remove_satellite_scanner_camera(&mut self, satcat_id: SatCatId) {
        if let Some(mut n) = self.scanner_cam_nodes.remove(&satcat_id) {
            self.window.remove_node(&mut n);
        }
    }

    pub fn update_satellite_focus_camera(
        &mut self,
        satcat_id: SatCatId,
        viewing_volume_radius: f64,
        viewing_volume_height: f64,
        iso: &Isometry3<f64>,
    ) {
        self.focus_cam_nodes
            .entry(satcat_id)
            .and_modify(|n| {
                n.set_local_scale(
                    scale(viewing_volume_radius * 2.0),
                    scale(viewing_volume_height),
                    scale(viewing_volume_radius * 2.0),
                );
                let iso = scale_iso(iso);
                n.set_local_transformation(iso);
            })
            .or_insert_with(|| {
                let mut n = self
                    .window
                    .add_cone(scale(viewing_volume_radius), scale(viewing_volume_height));
                n.set_color(0.0, 1.0, 1.0);
                n.set_points_size(1.0);
                n.set_lines_width(1.0);
                n.set_surface_rendering_activation(false);
                let iso = scale_iso(iso);
                n.set_local_transformation(iso);
                n
            });
    }

    pub fn remove_satellite_focus_camera(&mut self, satcat_id: SatCatId) {
        if let Some(mut n) = self.focus_cam_nodes.remove(&satcat_id) {
            self.window.remove_node(&mut n);
        }
    }

    pub fn update_ir_event(
        &mut self,
        ground_truth_id: GroundTruthId,
        pos_w: &Vector3<f64>,
        _vel_w: &Vector3<f64>,
    ) {
        self.ir_event_ndoes
            .entry(ground_truth_id)
            .and_modify(|n| {
                n.set_local_translation(scale_v3(pos_w).into());
            })
            .or_insert_with(|| {
                let mut n = self.window.add_sphere(0.1);
                n.set_color(1.0, 0.0, 0.0);
                n.set_local_translation(scale_v3(pos_w).into());
                n
            });
    }

    pub fn remove_ir_event(&mut self, ground_truth_id: GroundTruthId) {
        if let Some(mut n) = self.ir_event_ndoes.remove(&ground_truth_id) {
            self.window.remove_node(&mut n);
        }
    }

    pub fn update_relay_ground_station(&mut self, id: GroundStationId, pos_w: &Vector3<f64>) {
        self.relay_ground_station_nodes
            .entry(id)
            .and_modify(|n| {
                n.set_local_translation(scale_v3(pos_w).into());
            })
            .or_insert_with(|| {
                let r = 50000.0;
                let h = 200000.0;

                // Principal axis aligned with y
                let ref_point = Vector3::y() * (h / 2.0);

                // Rotate the principal axis so the cylinder is orthogonal to the Earth surface
                let rotation =
                    UnitQuaternion::rotation_between(&ref_point, pos_w).expect("Bad rotation");
                let iso = Isometry3 {
                    rotation,
                    translation: (*pos_w).into(),
                };

                let mut n = self.window.add_cylinder(scale(r), scale(h));
                n.set_color(1.0, 0.8, 0.4);
                n.set_local_transformation(scale_iso(&iso));
                n
            });
    }
}

impl RenderContext {
    fn swap(self) -> Self {
        match self {
            RenderContext::Sim => RenderContext::Fsm,
            RenderContext::Fsm => RenderContext::Sim,
        }
    }

    fn as_str(&self) -> &'static str {
        match self {
            RenderContext::Sim => "SIM",
            RenderContext::Fsm => "FSM",
        }
    }
}

fn scale_iso(inp: &Isometry3<f64>) -> Isometry3<f32> {
    let Isometry3 {
        rotation,
        translation,
    } = inp;

    let coords = scale_v4(&rotation.coords);
    let translation = scale_v3(&translation.vector).into();

    Isometry3 {
        rotation: UnitQuaternion::from_quaternion(Quaternion { coords }),
        translation,
    }
}

fn scale_v4(inp: &Vector4<f64>) -> Vector4<f32> {
    let s = inp.scale(1.0 / SYSTEM_SCALE);
    Vector4::new(s[0] as _, s[1] as _, s[2] as _, s[3] as _)
}

fn scale_v3(inp: &Vector3<f64>) -> Vector3<f32> {
    let s = inp.scale(1.0 / SYSTEM_SCALE);
    Vector3::new(s.x as _, s.y as _, s.z as _)
}

fn scale(inp: f64) -> f32 {
    (inp / SYSTEM_SCALE) as f32
}

// Mesh, color, texture
#[allow(clippy::type_complexity)]
struct ModelResources(Vec<(Rc<RefCell<Mesh>>, [f32; 3], Option<Rc<Texture>>)>);

fn create_sat_model_node(win: &mut Window, model_resources: &Option<ModelResources>) -> SceneNode {
    if let Some(model_resources) = model_resources.as_ref() {
        let alignment_iso = Isometry3 {
            rotation: na::one(),
            //rotation: UnitQuaternion::from_scaled_axis(Vector3::y() * std::f32::consts::FRAC_PI_2),
            translation: na::one(),
        };
        let mut root_node = SceneNode::new(SAT_OBJ_SCALE, alignment_iso, None);
        let def_mtl = MaterialManager::get_global_manager(|mm| mm.get_default());
        let def_tex = TextureManager::get_global_manager(|tm| tm.get_default());

        for (mesh, color, texture) in model_resources.0.iter() {
            let mut object = Object::new(
                mesh.clone(),
                1.0,
                1.0,
                1.0,
                def_tex.clone(),
                def_mtl.clone(),
            );
            object.set_color(color[0], color[1], color[2]);
            if let Some(tex) = texture {
                object.set_texture(tex.clone());
            }
            let _ = root_node.add_object(Vector3::from_element(1.0), na::one(), object);
        }

        win.scene_mut().add_child(root_node.clone());
        root_node
    } else {
        let mut n = win.add_sphere(0.32);
        n.set_color(SAT_NOMINAL_RGB[0], SAT_NOMINAL_RGB[1], SAT_NOMINAL_RGB[2]);
        n
    }
}
