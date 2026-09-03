/// One acquisition frame in device counts. Conversion to engineering units
/// is the host's job.
#[derive(Copy, Clone, Debug, Default)]
pub struct SensorFrame {
    pub tick: u32,
    pub pos: u16,
    pub current: u16,
    pub current_trough: u16,
    pub vmotor_a: u16,
    pub vmotor_a_trough: u16,
    pub vmotor_b: u16,
    pub vmotor_b_trough: u16,
    pub vcal: u16,
}
