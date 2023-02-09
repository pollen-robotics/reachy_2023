pub struct MotorModel {
    pub offset: f64,
    pub is_direct: bool,
    pub reduction: f64,
}

impl MotorModel {
    pub fn to_global_position(&self, pos: f64) -> f64 {
        (if self.is_direct { pos } else { -pos } - self.offset) / self.reduction
    }
    pub fn to_local_position(&self, pos: f64) -> f64 {
        (pos * self.reduction + self.offset) * if self.is_direct { 1.0 } else { -1.0 }
    }

    pub fn to_global_speed(&self, speed: f64) -> f64 {
        (if self.is_direct { speed } else { -speed }) / self.reduction
    }
    pub fn to_local_speed(&self, speed: f64) -> f64 {
        (speed * self.reduction) * if self.is_direct { 1.0 } else { -1.0 }
    }
    pub fn to_global_max_speed(&self, speed: f64) -> f64 {
        speed / self.reduction.abs()
    }
    pub fn to_local_max_speed(&self, speed: f64) -> f64 {
        speed * self.reduction.abs()
    }

    pub fn to_global_load(&self, load: f64) -> f64 {
        (if self.is_direct { load } else { -load }) / self.reduction
    }
    pub fn to_local_load(&self, load: f64) -> f64 {
        (load * self.reduction) * if self.is_direct { 1.0 } else { -1.0 }
    }
    pub fn to_global_max_load(&self, load: f64) -> f64 {
        load
    }
    pub fn to_local_max_load(&self, load: f64) -> f64 {
        load
    }
}
